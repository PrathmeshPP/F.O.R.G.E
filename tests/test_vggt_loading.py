"""
F.O.R.G.E — Phase 1D: VGGT-Omega Checkpoint Loading and Minimal Inference Validation Test Script.
"""
import os
import sys
import time
import numpy as np
from PIL import Image
import torch

CHECKPOINT_PATH = os.environ.get(
    "VGGT_OMEGA_CHECKPOINT",
    r"C:\Users\25beevdt025\Desktop\SIH\models\vggt_omega_1b_512.pt"
)
WORKSPACE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEST_INPUT_DIR = os.path.join(WORKSPACE_PATH, "tests", "test_inputs")

def format_bytes(b: int) -> str:
    return f"{b / (1024**2):.2f} MB ({b / (1024**3):.2f} GB)"

def generate_synthetic_images(output_dir: str, num_images: int = 2, size: tuple = (512, 512)) -> list[str]:
    os.makedirs(output_dir, exist_ok=True)
    paths = []
    for i in range(num_images):
        w, h = size
        x = np.linspace(0, 255, w, dtype=np.uint8)
        y = np.linspace(0, 255, h, dtype=np.uint8)
        xx, yy = np.meshgrid(x, y)
        r = ((xx + i * 40) % 256).astype(np.uint8)
        g = ((yy + i * 60) % 256).astype(np.uint8)
        b = ((xx // 2 + yy // 2 + i * 80) % 256).astype(np.uint8)
        img_arr = np.stack([r, g, b], axis=-1)
        img = Image.fromarray(img_arr)
        img_path = os.path.join(output_dir, f"synth_view_{i+1:02d}.png")
        img.save(img_path)
        paths.append(img_path)
    return paths

def main():
    print("=" * 60)
    print("F.O.R.G.E — Phase 1D: VGGT-Omega Loading & Inference Test")
    print("=" * 60)

    # 1. Preflight
    print("\n[STEP 1] Preflight System Check")
    print(f"Workspace path: {WORKSPACE_PATH}")
    print(f"Current working dir: {os.getcwd()}")
    print(f"Python interpreter: {sys.executable}")
    print(f"Python version: {sys.version.split()[0]}")
    print(f"PyTorch version: {torch.__version__}")

    cuda_avail = torch.cuda.is_available()
    print(f"CUDA available: {cuda_avail}")
    if not cuda_avail:
        print("FATAL: CUDA is not available. Stopping.")
        sys.exit(1)

    device_name = torch.cuda.get_device_name(0)
    print(f"GPU device: {device_name}")
    free_b, total_b = torch.cuda.mem_get_info(0)
    print(f"Total GPU memory: {format_bytes(total_b)}")
    print(f"Free GPU memory:  {format_bytes(free_b)}")

    alloc_pre = torch.cuda.memory_allocated(0)
    res_pre = torch.cuda.memory_reserved(0)
    print(f"GPU memory allocated before loading: {format_bytes(alloc_pre)}")
    print(f"GPU memory reserved before loading:  {format_bytes(res_pre)}")

    # 2. Modules import
    print("\n[STEP 2] Importing VGGT-Omega modules")
    try:
        from vggt_omega.models import VGGTOmega
        from vggt_omega.utils.load_fn import load_and_preprocess_images
        from vggt_omega.utils.pose_enc import encoding_to_camera
        print("Successfully imported VGGTOmega and helper utilities.")
    except Exception as e:
        print(f"FATAL: Failed to import VGGT-Omega modules: {e}")
        sys.exit(1)

    # Checkpoint verification
    if not os.path.isfile(CHECKPOINT_PATH):
        print(f"FATAL: Checkpoint file not found: {CHECKPOINT_PATH}")
        sys.exit(1)
    ckpt_size = os.path.getsize(CHECKPOINT_PATH)
    print(f"Verified checkpoint path: {CHECKPOINT_PATH}")
    print(f"Checkpoint file size: {format_bytes(ckpt_size)}")

    # 3. Model Loading
    print("\n[STEP 3] Loading VGGT-Omega Checkpoint into GPU")
    load_start = time.perf_counter()
    try:
        print("Instantiating VGGTOmega model architecture...")
        model = VGGTOmega()
        model.eval()

        print("Loading weights from disk to CPU memory...")
        state_dict = torch.load(CHECKPOINT_PATH, map_location="cpu")
        model.load_state_dict(state_dict)

        print("Moving model weights to CUDA (device: cuda:0)...")
        model = model.to("cuda")
        torch.cuda.synchronize()
        load_time = time.perf_counter() - load_start
        load_success = True
        print(f"Model loaded and transferred to GPU in {load_time:.2f} seconds.")
    except torch.cuda.OutOfMemoryError as oom:
        print(f"FATAL: CUDA Out-Of-Memory during model loading!")
        print(f"Current allocation: {format_bytes(torch.cuda.memory_allocated(0))}")
        print(f"Reserved allocation: {format_bytes(torch.cuda.memory_reserved(0))}")
        print(f"Error detail: {oom}")
        sys.exit(2)
    except Exception as e:
        print(f"FATAL: Model loading failed with exception: {e}")
        sys.exit(1)

    alloc_post_load = torch.cuda.memory_allocated(0)
    res_post_load = torch.cuda.memory_reserved(0)
    print(f"GPU memory allocated after loading: {format_bytes(alloc_post_load)} (+{format_bytes(alloc_post_load - alloc_pre)})")
    print(f"GPU memory reserved after loading:  {format_bytes(res_post_load)} (+{format_bytes(res_post_load - res_pre)})")
    print("Model loading result: SUCCESS")

    # 4. Prepare minimal test input
    print("\n[STEP 4] Preparing Minimal Synthetic Input")
    image_paths = generate_synthetic_images(TEST_INPUT_DIR, num_images=2, size=(512, 512))
    for p in image_paths:
        print(f"  Created test image: {p} (size: 512x512 RGB)")

    print("Preprocessing images with load_and_preprocess_images (image_resolution=512)...")
    input_tensor = load_and_preprocess_images(image_paths, image_resolution=512).to("cuda")
    print(f"Preprocessed input tensor shape: {tuple(input_tensor.shape)} (Views, Channels, Height, Width)")
    print(f"Input tensor dtype: {input_tensor.dtype}, device: {input_tensor.device}")

    # 5. Run inference pass
    print("\n[STEP 5] Running Inference Pass")
    torch.cuda.reset_peak_memory_stats(0)
    torch.cuda.synchronize()

    infer_start = time.perf_counter()
    inference_success = False
    predictions = None

    try:
        with torch.inference_mode():
            predictions = model(input_tensor)
        torch.cuda.synchronize()
        infer_time = time.perf_counter() - infer_start
        inference_success = True
        print(f"Inference pass completed successfully in {infer_time:.3f} seconds.")
    except torch.cuda.OutOfMemoryError as oom:
        print(f"FATAL: CUDA Out-Of-Memory during inference!")
        print(f"Current allocation: {format_bytes(torch.cuda.memory_allocated(0))}")
        print(f"Reserved allocation: {format_bytes(torch.cuda.memory_reserved(0))}")
        print(f"Peak allocation: {format_bytes(torch.cuda.max_memory_allocated(0))}")
        print(f"Error detail: {oom}")
        sys.exit(3)
    except Exception as e:
        print(f"FATAL: Inference pass failed with exception: {e}")
        sys.exit(1)

    # Memory metrics
    peak_alloc = torch.cuda.max_memory_allocated(0)
    peak_reserved = torch.cuda.max_memory_reserved(0)
    print(f"Peak GPU memory allocated during inference: {format_bytes(peak_alloc)}")
    print(f"Peak GPU memory reserved during inference:  {format_bytes(peak_reserved)}")

    # Output inspection
    print("\n[STEP 6] Inspecting Output Predictions Structure")
    print(f"Output type: {type(predictions)}")
    all_finite = True
    if isinstance(predictions, dict):
        for k, v in predictions.items():
            if isinstance(v, torch.Tensor):
                finite = torch.isfinite(v).all().item()
                if not finite:
                    all_finite = False
                print(f"  - Key: '{k}': Tensor shape={tuple(v.shape)}, dtype={v.dtype}, device={v.device}, all_finite={finite}")
            else:
                print(f"  - Key: '{k}': Type={type(v)}")

    # Camera pose extraction verification
    if "pose_enc" in predictions:
        try:
            extrinsic, intrinsic = encoding_to_camera(predictions["pose_enc"], image_size_hw=(512, 512))
            print(f"Extracted extrinsic camera shape: {tuple(extrinsic.shape)}")
            print(f"Extracted intrinsic camera shape: {tuple(intrinsic.shape)}")
        except Exception as e:
            print(f"Camera pose conversion notice: {e}")

    print("\n" + "=" * 60)
    if inference_success and all_finite:
        print("PHASE 1D VALIDATION RESULT: PASS")
    else:
        print("PHASE 1D VALIDATION RESULT: FAIL")
    print("=" * 60)

if __name__ == "__main__":
    main()
