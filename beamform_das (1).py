#!/usr/bin/env python3
"""Calibrated 2-D monostatic/bistatic synthetic-aperture delay-and-sum.

RF rows are successive mechanical positions, columns are time samples.
All API distances are metres, times seconds, rates samples/second. Sample zero
occurred at t0_s relative to transmission. Positive system_delay_s means the
recorded echo is later than the ideal acoustic flight time. The SAME echo delay
must not be included in both calibration fields.

Run ``python beamform_das.py simulate --output-dir results/demo`` for a complete
example; ``python beamform_das.py reconstruct acquisition.npz`` for real data.
This is a point-phase-centre, homogeneous-medium model, not a finite-piston
field simulator or a simultaneous plane-wave acquisition.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
import warnings

import numpy as np
from scipy import signal
from eclypse_z7 import DEFAULT_FS_HZ, CONTROLLER_BITS


def _positive(value: float, name: str) -> float:
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _per_line(value, n: int, name: str) -> np.ndarray:
    a = np.asarray(value, dtype=np.float64)
    if a.ndim == 0:
        a = np.full(n, a.item())
    if a.shape != (n,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be a finite scalar or shape ({n},)")
    return a


@dataclass
class Acquisition:
    """Receive and transmit phase centres; omitted TX positions equal RX."""
    fs_hz: float
    rx_positions_m: np.ndarray
    c_m_s: float = 1540.0
    t0_s: float | np.ndarray = 0.0
    system_delay_s: float | np.ndarray = 0.0
    tx_positions_m: np.ndarray | None = None

    def __post_init__(self):
        self.fs_hz = _positive(self.fs_hz, "fs_hz")
        self.c_m_s = _positive(self.c_m_s, "c_m_s")
        for name in ("rx_positions_m", "tx_positions_m"):
            value = getattr(self, name)
            if value is None:
                value = self.rx_positions_m.copy()
            a = np.asarray(value, dtype=np.float64)
            if a.ndim != 2 or a.shape[1] != 2 or len(a) < 1 or not np.all(np.isfinite(a)):
                raise ValueError(f"{name} must be finite shape (positions, 2), columns x,z")
            setattr(self, name, a.copy())
        if self.tx_positions_m.shape != self.rx_positions_m.shape:
            raise ValueError("TX and RX must have one phase centre for each RF row")
        n = len(self.rx_positions_m)
        self.t0_s = _per_line(self.t0_s, n, "t0_s")
        self.system_delay_s = _per_line(self.system_delay_s, n, "system_delay_s")

    @property
    def n_positions(self):
        return len(self.rx_positions_m)


@dataclass
class Grid:
    """Image axes, increasing x and z; images have shape (len(z),len(x))."""
    x_m: np.ndarray
    z_m: np.ndarray

    def __post_init__(self):
        for name in ("x_m", "z_m"):
            a = np.asarray(getattr(self, name), dtype=np.float64)
            if a.ndim != 1 or len(a) < 1 or not np.all(np.isfinite(a)):
                raise ValueError(f"{name} must be a nonempty finite vector")
            if len(a) > 1 and np.any(np.diff(a) <= 0):
                raise ValueError(f"{name} must strictly increase")
            setattr(self, name, a.copy())
        if np.any(self.z_m < 0):
            raise ValueError("This image convention requires nonnegative depth")

    @property
    def shape(self):
        return (len(self.z_m), len(self.x_m))

    def pixels(self):
        x, z = np.meshgrid(self.x_m, self.z_m, indexing="xy")
        return np.column_stack((x.ravel(), z.ravel()))


@dataclass
class DASOptions:
    interpolation: str = "linear"
    apodization: str = "rectangular"
    normalize: bool = False
    f_number: float | None = None
    max_angle_deg: float | None = None
    coherence_weight: bool = False
    dynamic_range_db: float = 60.0
    chunk_pixels: int = 4096

    def __post_init__(self):
        if self.interpolation not in ("linear", "nearest"):
            raise ValueError("interpolation must be linear or nearest")
        if self.apodization not in ("rectangular", "hann"):
            raise ValueError("apodization must be rectangular or hann")
        if self.f_number is not None:
            self.f_number = _positive(self.f_number, "f_number")
        if self.max_angle_deg is not None and not (0 < self.max_angle_deg < 90):
            raise ValueError("max_angle_deg must lie between 0 and 90")
        _positive(self.dynamic_range_db, "dynamic_range_db")
        if not isinstance(self.chunk_pixels, (int, np.integer)) or self.chunk_pixels < 1:
            raise ValueError("chunk_pixels must be a positive integer")


def centred_positions(n: int = 25, pitch_m: float = 1e-3) -> np.ndarray:
    if not isinstance(n, (int, np.integer)) or n < 1:
        raise ValueError("n must be a positive integer")
    _positive(pitch_m, "pitch_m")
    return np.column_stack(((np.arange(n) - (n - 1) / 2) * pitch_m, np.zeros(n)))


def sample_coordinates(pixels_m, acquisition: Acquisition) -> np.ndarray:
    """Return fractional, zero-based RF indices, shape (pixels,positions)."""
    p = np.asarray(pixels_m, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 2 or not np.all(np.isfinite(p)):
        raise ValueError("pixels_m must have finite shape (pixels,2)")
    dr = np.linalg.norm(p[:, None, :] - acquisition.rx_positions_m[None, :, :], axis=-1)
    dt = np.linalg.norm(p[:, None, :] - acquisition.tx_positions_m[None, :, :], axis=-1)
    return acquisition.fs_hz * ((dt + dr) / acquisition.c_m_s
                              + acquisition.system_delay_s - acquisition.t0_s)


def required_samples(acquisition: Acquisition, grid: Grid, tail_s=0.0) -> int:
    """Count includes sample zero and the last sample supporting interpolation.

    Corner distances bound this rectangular grid. tail_s is a user-chosen pulse
    tail/calibration margin, not an assumed transducer ring-down duration.
    """
    if not np.isfinite(tail_s) or tail_s < 0:
        raise ValueError("tail_s must be nonnegative")
    p = np.array([(x,z) for x in (grid.x_m[0],grid.x_m[-1])
                  for z in (grid.z_m[0],grid.z_m[-1])])
    last = np.max(sample_coordinates(p, acquisition)) + tail_s * acquisition.fs_hz
    return max(2, int(math.ceil(last)) + 1)


def decode_adc(words, bits=14, encoding="twos-complement") -> np.ndarray:
    """Decode right-aligned ADC words or validate already sign-extended samples.

    For twos-complement/offset-binary, upper padding bits are ignored. No
    automatic format guessing is performed. Output is signed int16 (bits<=16).
    """
    a = np.asarray(words)
    if not np.issubdtype(a.dtype, np.integer) or not 2 <= bits <= 16:
        raise ValueError("ADC words must be integer and bits must be in [2,16]")
    sign = 1 << (bits - 1)
    if encoding == "signed":
        v = a.astype(np.int64)
        if np.any((v < -sign) | (v >= sign)):
            raise ValueError("Signed ADC samples exceed the selected bit width")
    else:
        v = a.astype(np.int64) & ((1 << bits) - 1)
        if encoding == "twos-complement":
            v = (v ^ sign) - sign
        elif encoding == "offset-binary":
            v = v - sign
        else:
            raise ValueError("encoding must be signed, twos-complement or offset-binary")
    return v.astype(np.int16)


def validate_rf(rf, acquisition: Acquisition) -> np.ndarray:
    a = np.asarray(rf)
    if (a.ndim != 2 or a.shape[0] != acquisition.n_positions or a.shape[1] < 2
            or not np.issubdtype(a.dtype, np.number) or np.iscomplexobj(a)
            or not np.all(np.isfinite(a))):
        raise ValueError("rf must be finite real array (positions,samples), samples>=2")
    # Integer inputs here must ALREADY be signed/centred ADC samples.
    if np.issubdtype(a.dtype, np.unsignedinteger):
        raise ValueError("Unsigned ADC words require explicit decode_adc before beamforming")
    return a.astype(np.float64, copy=True)


def preprocess_rf(rf, acquisition: Acquisition, *, remove_dc=False,
                  bandpass_hz=None, decimation=1):
    """Optional offline, zero-phase filter and anti-aliased integer decimation.

    scipy.resample_poly aligns output sample zero with input sample zero. ADC/
    analogue latency is still calibrated separately. Edge filter transients
    require record margin; zero-phase filtering can leak TX artefacts backwards.
    """
    a = validate_rf(rf, acquisition)
    if remove_dc:
        a -= a.mean(axis=1, keepdims=True)
    if bandpass_hz is not None:
        low, high = bandpass_hz
        if not (0 < low < high < acquisition.fs_hz / 2):
            raise ValueError("bandpass must satisfy 0 < low < high < Nyquist")
        sos = signal.butter(4, [low, high], btype="bandpass", fs=acquisition.fs_hz, output="sos")
        a = signal.sosfiltfilt(sos, a, axis=1)
    if not isinstance(decimation, (int, np.integer)) or decimation < 1:
        raise ValueError("decimation must be a positive integer")
    if decimation > 1:
        a = signal.resample_poly(a, 1, decimation, axis=1, padtype="line")
    acq = Acquisition(acquisition.fs_hz / decimation, acquisition.rx_positions_m,
                      acquisition.c_m_s, acquisition.t0_s, acquisition.system_delay_s,
                      acquisition.tx_positions_m)
    return a, acq


def _weights(p, acq, opt):
    dx = p[:,None,0] - acq.rx_positions_m[None,:,0]
    dz = p[:,None,1] - acq.rx_positions_m[None,:,1]
    w = np.ones(dx.shape)
    if opt.f_number is None:
        if opt.apodization == "hann" and acq.n_positions > 2:
            # This taper is by sorted aperture position, not input row order.
            order = np.argsort(acq.rx_positions_m[:,0], kind="stable")
            base = np.zeros(acq.n_positions)
            base[order] = np.hanning(acq.n_positions)
            w *= base
    else:
        half = np.maximum(dz, 0) / (2 * opt.f_number)
        inside = (dz > 0) & (np.abs(dx) <= half)
        w = inside.astype(float)
        if opt.apodization == "hann":
            ratio = np.divide(dx, half, out=np.zeros_like(dx), where=half>0)
            w *= .5 * (1 + np.cos(np.pi * np.clip(ratio, -1, 1)))
    if opt.max_angle_deg is not None:
        # Apply directivity gate to BOTH outward and return legs.
        for pos in (acq.tx_positions_m, acq.rx_positions_m):
            angle = np.abs(np.arctan2(p[:,None,0]-pos[None,:,0],
                                     p[:,None,1]-pos[None,:,1]))
            w *= angle <= np.deg2rad(opt.max_angle_deg)
    return w


def _gather(lines, s, interpolation):
    n = lines.shape[1]
    valid = (s >= 0) & (s <= n-1)
    safe = np.clip(s, 0, n-1)
    rows = np.arange(lines.shape[0])[None,:]
    if interpolation == "nearest":
        # Deliberate half-up rule, not NumPy's ties-to-even round.
        index = np.floor(safe+.5).astype(np.int64)
        v = lines[rows, index]
    else:
        i0 = np.floor(safe).astype(np.int64)
        a = safe-i0
        i1 = np.minimum(i0+1, n-1)
        v = (1-a)*lines[rows,i0] + a*lines[rows,i1]
    return np.where(valid, v, 0), valid


def log_compress(envelope, dynamic_range_db=60.0, reference=None):
    _positive(dynamic_range_db, "dynamic_range_db")
    a = np.asarray(envelope, dtype=float)
    if not np.all(np.isfinite(a)) or np.any(a < 0):
        raise ValueError("envelope must be finite and nonnegative")
    peak = float(np.max(a)) if reference is None else _positive(reference, "reference")
    if peak == 0:
        return np.full_like(a, -dynamic_range_db)
    ratio = np.maximum(a/peak, 10**(-dynamic_range_db/20))
    return np.clip(20*np.log10(ratio), -dynamic_range_db, 0)


def beamform_das(rf, acquisition: Acquisition, grid: Grid, options=None):
    """Coherently interpolate and sum analytic RF, then take its magnitude.

    ``signed_rf`` is the real part of the SAME sum, useful for FPGA comparison.
    ``valid_count`` counts positive-weight, in-record contributors. Unsupported
    pixels are zero, not wrapped/clamped edge samples. CF uses nonnegative
    aperture weights and is optional; unweighted DAS is the reference.
    """
    opt = DASOptions() if options is None else options
    real = validate_rf(rf, acquisition)
    analytic = signal.hilbert(real, axis=1)
    pixels = grid.pixels()
    output = np.zeros(len(pixels), np.complex128)
    count = np.zeros(len(pixels), np.int32)
    cf = np.zeros(len(pixels))
    weight_sum = np.zeros(len(pixels))
    for start in range(0, len(pixels), opt.chunk_pixels):
        stop = min(start+opt.chunk_pixels, len(pixels))
        p = pixels[start:stop]
        v, valid = _gather(analytic, sample_coordinates(p, acquisition), opt.interpolation)
        w = _weights(p, acquisition, opt) * valid
        summed = np.sum(w*v, axis=1)
        sw = w.sum(axis=1)
        den = sw*np.sum(w*np.abs(v)**2, axis=1)
        coherent = np.divide(np.abs(summed)**2, den, out=np.zeros(len(p)), where=den>0)
        cf[start:stop] = np.clip(coherent, 0, 1)
        if opt.normalize:
            summed = np.divide(summed, sw, out=np.zeros_like(summed), where=sw>0)
        output[start:stop] = summed
        count[start:stop] = np.sum(w>0, axis=1)
        weight_sum[start:stop] = sw
    envelope = np.abs(output)
    if opt.coherence_weight:
        envelope *= cf
    reshape = lambda a: a.reshape(grid.shape)
    return {"analytic_das": reshape(output), "signed_rf": reshape(output.real),
            "envelope": reshape(envelope), "bmode_db": reshape(log_compress(envelope,opt.dynamic_range_db)),
            "valid_count": reshape(count), "weight_sum": reshape(weight_sum),
            "coherence_factor": reshape(cf), "x_m": grid.x_m, "z_m": grid.z_m}


def export_delay_table(acquisition: Acquisition, grid: Grid, n_samples: int,
                       frac_bits=12, interpolation="linear"):
    """Pixel-major LUT for the OPTIONAL rectangular, unnormalised RTL core.

    For linear delay, fractional coefficients round half-up in Q0.frac_bits;
    a rounded coefficient of unity carries into the integer address. Out-of-
    record positions get valid=False and harmless zero addresses. RAM/DDR must
    respect validity. Do not store a full 241x401x25 LUT in Z7020 BRAM.
    """
    if n_samples < 2 or not 1 <= frac_bits <= 16:
        raise ValueError("n_samples>=2 and 1<=frac_bits<=16 required")
    if interpolation not in ("linear","nearest"):
        raise ValueError("interpolation must be linear or nearest")
    s = sample_coordinates(grid.pixels(), acquisition)
    valid = (s>=0) & (s<=n_samples-1)
    safe = np.clip(s,0,n_samples-1)
    scale = 1 << frac_bits
    if interpolation == "nearest":
        i0 = np.floor(safe+.5).astype(np.int64)
        q = np.zeros_like(i0)
    else:
        fixed = np.floor(safe*scale+.5).astype(np.int64)
        i0, q = fixed//scale, fixed%scale
    i1 = np.minimum(i0+1,n_samples-1)
    return {"addr0":np.where(valid,i0,0).astype(np.uint32),
            "addr1":np.where(valid,i1,0).astype(np.uint32),
            "fraction":q.astype(np.uint16), "valid":valid,
            "frac_bits":np.array(frac_bits), "grid_shape":np.array(grid.shape),
            "x_m":grid.x_m,"z_m":grid.z_m}


def simulate_point_targets(acquisition: Acquisition, n_samples: int, targets,
                           centre_hz=2.25e6, sigma_s=.3e-6, noise_std=0.0, seed=7):
    """Simple coherent Gaussian RF pulses; not a finite-element acoustic model.

    targets rows are x_m,z_m,amplitude. No field, attenuation or speckle model.
    This independent forward time-domain model tests indexing/localisation.
    """
    _positive(centre_hz,"centre_hz"); _positive(sigma_s,"sigma_s")
    if centre_hz >= acquisition.fs_hz/2 or n_samples<2 or noise_std<0:
        raise ValueError("Invalid synthetic sampling/pulse/noise settings")
    result = np.zeros((acquisition.n_positions,n_samples))
    for x,z,amp in targets:
        target = np.array([x,z])
        if not np.all(np.isfinite([x,z,amp])):
            raise ValueError("Targets must be finite")
        for k,(tx,rx) in enumerate(zip(acquisition.tx_positions_m,acquisition.rx_positions_m)):
            arrival = (np.linalg.norm(target-tx)+np.linalg.norm(target-rx))/acquisition.c_m_s
            arrival += acquisition.system_delay_s[k]
            t = acquisition.t0_s[k] + np.arange(n_samples)/acquisition.fs_hz - arrival
            result[k] += amp*np.exp(-.5*(t/sigma_s)**2)*np.cos(2*np.pi*centre_hz*t)
    if noise_std:
        result += np.random.default_rng(seed).normal(0,noise_std,result.shape)
    return result


def save_acquisition(path, rf, acquisition: Acquisition, *, metadata=None):
    validate_rf(rf,acquisition)
    np.savez_compressed(path,rf=rf,fs_hz=acquisition.fs_hz,c_m_s=acquisition.c_m_s,
                        rx_positions_m=acquisition.rx_positions_m,
                        tx_positions_m=acquisition.tx_positions_m,t0_s=acquisition.t0_s,
                        system_delay_s=acquisition.system_delay_s,
                        metadata_json=np.array(json.dumps(metadata or {},sort_keys=True)))


def load_acquisition(path, *, encoding="signed", bits=CONTROLLER_BITS):
    """NPZ contains rf, fs_hz, rx_positions_m (or x_positions_m), plus calibration."""
    with np.load(path,allow_pickle=False) as data:
        if "rx_positions_m" in data:
            positions = data["rx_positions_m"]
        else:
            xs = data["x_positions_m"]
            positions = np.column_stack((xs,np.zeros(len(xs))))
        acq = Acquisition(float(data["fs_hz"]), positions,
                          float(data["c_m_s"]) if "c_m_s" in data else 1540.,
                          data["t0_s"] if "t0_s" in data else 0.,
                          data["system_delay_s"] if "system_delay_s" in data else 0.,
                          data["tx_positions_m"] if "tx_positions_m" in data else None)
        rf = data["rf"]
        if np.issubdtype(rf.dtype,np.integer):
            rf = decode_adc(rf,bits,encoding)
        elif encoding != "signed":
            raise ValueError("Raw-word ADC encodings require integer RF data")
        return validate_rf(rf,acq),acq


def plot_bmode(result, path, dynamic_range_db=60.):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,ax = plt.subplots(figsize=(7,7),layout="constrained")
    x,z = result["x_m"]*1e3,result["z_m"]*1e3
    if len(x)<2 or len(z)<2:
        raise ValueError("Plot requires at least two samples on both image axes")
    mesh = ax.pcolormesh(x,z,result["bmode_db"],cmap="gray",vmin=-dynamic_range_db,
                         vmax=0,shading="nearest")
    ax.invert_yaxis(); ax.set_aspect("equal")
    ax.set(xlabel="Lateral position (mm)",ylabel="Depth (mm)",title="Synthetic-aperture DAS")
    fig.colorbar(mesh,ax=ax,label="Amplitude relative to image peak (dB)")
    fig.savefig(path,dpi=180); plt.close(fig)


def _cli():
    parser = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command",required=True)
    sim = sub.add_parser("simulate",help="Generate a reproducible three-target demonstration")
    rec = sub.add_parser("reconstruct",help="Reconstruct calibrated NPZ RF data")
    lut = sub.add_parser("export-lut",help="Export pixel-major integer/fractional delay descriptors")
    for p in (sim,rec,lut):
        p.add_argument("--output-dir",type=Path,default=Path("results"))
        p.add_argument("--x-range-mm",nargs=2,type=float,default=[-12,12])
        p.add_argument("--z-range-mm",nargs=2,type=float,default=[10,50])
        p.add_argument("--nx",type=int,default=241); p.add_argument("--nz",type=int,default=401)
        p.add_argument("--interpolation",choices=["nearest","linear"],default="linear")
    for p in (rec,lut):
        p.add_argument("input",type=Path)
        p.add_argument("--encoding",choices=["signed","twos-complement","offset-binary"],default="signed")
        p.add_argument("--adc-bits",type=int,default=CONTROLLER_BITS,
                       help="Stored word width; calibrated Zmod output is 16 bits")
    sim.add_argument("--fs-mhz",type=float,default=DEFAULT_FS_HZ/1e6)
    sim.add_argument("--pitch-mm",type=float,default=1.)
    sim.add_argument("--npositions",type=int,default=25)
    sim.add_argument("--record-us",type=float,default=80.)
    sim.add_argument("--c-m-s",type=float,default=1540.)
    for p in (sim,rec):
        p.add_argument("--apodization",choices=["rectangular","hann"],default="rectangular")
        p.add_argument("--normalize",action="store_true")
        p.add_argument("--f-number",type=float)
        p.add_argument("--max-angle-deg",type=float)
        p.add_argument("--coherence-weight",action="store_true")
        p.add_argument("--dynamic-range-db",type=float,default=60.)
        p.add_argument("--remove-dc",action="store_true")
        p.add_argument("--bandpass-mhz",type=float,nargs=2)
        p.add_argument("--decimation",type=int,default=1)
        p.add_argument("--chunk-pixels",type=int,default=4096)
        p.add_argument("--no-plot",action="store_true")
    lut.add_argument("--frac-bits",type=int,default=12)
    args = parser.parse_args()
    if args.nx<1 or args.nz<1:
        parser.error("nx and nz must be positive")
    grid = Grid(np.linspace(*np.array(args.x_range_mm)*1e-3,args.nx),
                np.linspace(*np.array(args.z_range_mm)*1e-3,args.nz))
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if args.command == "simulate":
        acq = Acquisition(args.fs_mhz*1e6,centred_positions(args.npositions,args.pitch_mm*1e-3),args.c_m_s)
        _positive(args.record_us,"record_us")
        n = int(math.ceil(args.record_us*1e-6*acq.fs_hz))
        targets = [(0,.03,1.),(-.004,.022,.6),(.006,.04,.8)]
        rf = simulate_point_targets(acq,n,targets,noise_std=.01)
        save_acquisition(args.output_dir/"acquisition.npz",rf,acq)
    else:
        rf,acq = load_acquisition(args.input,encoding=args.encoding,bits=args.adc_bits)
    minimum = required_samples(acq,grid)
    if rf.shape[1]<minimum:
        warnings.warn(f"Record has {rf.shape[1]} samples; full grid needs at least {minimum}. "
                      "Out-of-record contributors will be masked.")
    if args.command == "export-lut":
        table = export_delay_table(acq,grid,rf.shape[1],args.frac_bits,args.interpolation)
        np.savez_compressed(args.output_dir/"delay_table.npz",**table)
        print(f"Saved LUT for {len(grid.pixels())} pixels x {acq.n_positions} positions")
        return
    bp = None if args.bandpass_mhz is None else np.array(args.bandpass_mhz)*1e6
    rf,acq = preprocess_rf(rf,acq,remove_dc=args.remove_dc,bandpass_hz=bp,decimation=args.decimation)
    opts = DASOptions(args.interpolation,args.apodization,args.normalize,args.f_number,
                      args.max_angle_deg,args.coherence_weight,args.dynamic_range_db,args.chunk_pixels)
    result = beamform_das(rf,acq,grid,opts)
    np.savez_compressed(args.output_dir/"reconstruction.npz",**result)
    if not args.no_plot:
        plot_bmode(result,args.output_dir/"bmode.png",opts.dynamic_range_db)
    info = {"model":"bistatic two-leg point phase centres; TX=RX means monostatic SAFT",
            "fs_hz":acq.fs_hz,"c_m_s":acq.c_m_s,"rf_shape":list(rf.shape),
            "rx_positions_m":acq.rx_positions_m.tolist(),"tx_positions_m":acq.tx_positions_m.tolist(),
            "t0_s":acq.t0_s.tolist(),"system_delay_s":acq.system_delay_s.tolist(),
            "grid_shape":list(grid.shape),"options":asdict(opts),
            "unsupported_pixels":int(np.count_nonzero(result["valid_count"]==0)),
            "preprocessing":{"remove_dc":args.remove_dc,"bandpass_hz":None if bp is None else bp.tolist(),
                             "decimation":args.decimation}}
    (args.output_dir/"reconstruction.json").write_text(json.dumps(info,indent=2)+"\n")
    print(f"Saved reconstruction to {args.output_dir.resolve()}; "
          f"{info['unsupported_pixels']} pixels without contributing samples")


if __name__ == "__main__":
    _cli()
