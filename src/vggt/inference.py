"""
F.O.R.G.E — VGGT-Omega Inference Engine
Encapsulates validated model loading, pre-processing, inference pass, and camera decoding.
"""

import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import sys
import numpy as np
import torch

# Search candidate directories for external vggt_omega repository
for _candidate_vggt_dir in [
    r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega",
    r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main",
    r"C:\Users\25beevdt025\Desktop\coding\third_party\vggt_omega",
]:
    if os.path.isdir(_candidate_vggt_dir) and _candidate_vggt_dir not in sys.path:
        sys.path.insert(0, _candidate_vggt_dir)

try:
    from vggt_omega.models import VGGTOmega
    from vggt_omega.utils.load_fn import load_and_preprocess_images
    from vggt_omega.utils.pose_enc import encoding_to_camera
except ImportError:
    VGGTOmega = None
    load_and_preprocess_images = None
    encoding_to_camera = None

DEFAULT_CHECKPOINT_PATH = os.environ.get(
    "VGGT_OMEGA_CHECKPOINT",
    r"C:\Users\25beevdt025\Desktop\SIH\models\vggt_omega_1b_512.pt"
)


def load_vggt_model(checkpoint_path: str = DEFAULT_CHECKPOINT_PATH, device: str = "cuda:0") -> VGGTOmega:
    """
    Loads and initializes the validated VGGT-Omega model.

    Args:
        checkpoint_path: Path to the local .pt checkpoint.
        device: Target torch device (defaults to 'cuda:0').

    Returns:
        Evaluated VGGTOmega model on specified device.
    """
    if not os.path.isfile(checkpoint_path):
        raise FileNotFoundError(f"VGGT-Omega checkpoint not found at: {checkpoint_path}")

    if not torch.cuda.is_available() and "cuda" in device:
        raise RuntimeError("CUDA is required for GPU inference but is not available.")

    model = VGGTOmega()
    model.eval()

    state_dict = torch.load(checkpoint_path, map_location="cpu")
    model.load_state_dict(state_dict)

    model = model.to(device)
    if "cuda" in device:
        torch.cuda.synchronize()

    return model


class VGGTInferencePipeline:
    """
    High-level inference manager for VGGT-Omega.
    """

    def __init__(
        self,
        checkpoint_path: str = DEFAULT_CHECKPOINT_PATH,
        device: str = "cuda:0",
        image_resolution: int = 512
    ):
        self.checkpoint_path = checkpoint_path
        self.device = device
        self.image_resolution = image_resolution
        self.model: Optional[VGGTOmega] = None

    def initialize(self) -> float:
        """Loads model into GPU if not already initialized. Returns load time in seconds."""
        if self.model is None:
            t0 = time.perf_counter()
            self.model = load_vggt_model(self.checkpoint_path, self.device)
            load_time = time.perf_counter() - t0
            return load_time
        return 0.0

    def run_inference(self, image_paths: List[str]) -> Dict[str, Any]:
        """
        Executes a controlled forward pass on a list of image paths.

        Args:
            image_paths: Ordered list of image file paths.

        Returns:
            Dictionary containing decoded predictions, tensors, and metrics.
        """
        if not image_paths:
            raise ValueError("image_paths must contain at least one valid image path.")

        if self.model is None:
            self.initialize()

        # Measure baseline memory
        if "cuda" in self.device:
            torch.cuda.reset_peak_memory_stats(0)
            torch.cuda.synchronize()
            mem_pre_alloc = torch.cuda.memory_allocated(0)
            mem_pre_res = torch.cuda.memory_reserved(0)
        else:
            mem_pre_alloc = mem_pre_res = 0

        # Preprocessing
        prep_t0 = time.perf_counter()
        input_tensor = load_and_preprocess_images(
            image_paths,
            image_resolution=self.image_resolution
        ).to(self.device)
        prep_time = time.perf_counter() - prep_t0

        # Inference
        infer_t0 = time.perf_counter()
        with torch.inference_mode():
            raw_predictions = self.model(input_tensor)

        if "cuda" in self.device:
            torch.cuda.synchronize()
        infer_time = time.perf_counter() - infer_t0

        # Decode camera parameters
        pose_enc = raw_predictions.get("pose_enc")
        if pose_enc is not None:
            image_size_hw = raw_predictions["images"].shape[-2:] if "images" in raw_predictions else input_tensor.shape[-2:]
            extrinsics, intrinsics = encoding_to_camera(pose_enc, image_size_hw=image_size_hw)
        else:
            extrinsics = intrinsics = None

        # Collect memory stats
        if "cuda" in self.device:
            peak_alloc = torch.cuda.max_memory_allocated(0)
            peak_reserved = torch.cuda.max_memory_reserved(0)
        else:
            peak_alloc = peak_reserved = 0

        metrics = {
            "num_frames": len(image_paths),
            "image_resolution": self.image_resolution,
            "preprocessing_time_sec": round(prep_time, 4),
            "inference_time_sec": round(infer_time, 4),
            "total_time_sec": round(prep_time + infer_time, 4),
            "peak_vram_allocated_mb": round(peak_alloc / (1024**2), 2),
            "peak_vram_reserved_mb": round(peak_reserved / (1024**2), 2),
        }

        return {
            "extrinsics": extrinsics,
            "intrinsics": intrinsics,
            "depth": raw_predictions.get("depth"),
            "depth_conf": raw_predictions.get("depth_conf"),
            "pose_enc": pose_enc,
            "metrics": metrics,
            "input_tensor_shape": list(input_tensor.shape),
        }

    @staticmethod
    def save_predictions(results: Dict[str, Any], output_dir: str) -> Dict[str, str]:
        """
        Saves prediction arrays to disk (Numpy & PyTorch formats).
        """
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        saved_files = {}

        # Save depth and depth confidence
        depth = results.get("depth")
        depth_conf = results.get("depth_conf")
        if depth is not None and depth_conf is not None:
            depth_np = depth.squeeze(0).cpu().numpy()  # (N, H, W, 1)
            conf_np = depth_conf.squeeze(0).cpu().numpy()  # (N, H, W)
            depth_file = out_path / "depth_predictions.npz"
            np.savez_compressed(depth_file, depth=depth_np, confidence=conf_np)
            saved_files["depth_npz"] = str(depth_file.resolve())

        # Save camera extrinsics and intrinsics
        extrinsics = results.get("extrinsics")
        intrinsics = results.get("intrinsics")
        if extrinsics is not None and intrinsics is not None:
            ext_np = extrinsics.squeeze(0).cpu().numpy()  # (N, 3, 4)
            int_np = intrinsics.squeeze(0).cpu().numpy()  # (N, 3, 3)
            poses_file = out_path / "camera_poses.npz"
            np.savez_compressed(poses_file, extrinsics=ext_np, intrinsics=int_np)
            saved_files["camera_poses_npz"] = str(poses_file.resolve())

        # Save full PyTorch state
        pt_file = out_path / "predictions_raw.pt"
        torch_dict = {
            "extrinsics": extrinsics.cpu() if extrinsics is not None else None,
            "intrinsics": intrinsics.cpu() if intrinsics is not None else None,
            "depth": depth.cpu() if depth is not None else None,
            "depth_conf": depth_conf.cpu() if depth_conf is not None else None,
            "pose_enc": results.get("pose_enc").cpu() if results.get("pose_enc") is not None else None,
            "metrics": results.get("metrics"),
        }
        torch.save(torch_dict, pt_file)
        saved_files["raw_predictions_pt"] = str(pt_file.resolve())

        return saved_files
