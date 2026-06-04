# Containerized SysNav — build & deploy

Full GPU build of the VLM_ROS AlphaZ stack, run **headless** against a rosbag,
designed to deploy on AWS (EC2 GPU or AWS Batch). The container mirrors
`vlm_ros_alphaz.tmuxp.yaml` minus RViz and the manual SPACE-to-start.

## Files
- `Dockerfile` — CUDA 12.6.3 + ROS Jazzy, native SLAM deps, ML stack, baked model weights, full colcon build.
- `docker/entrypoint.sh` — pull bag from S3 → run system → push outputs to S3.
- `docker/run_system.sh` — headless launch supervisor (replaces tmuxp).
- `docker-compose.yml` — local GPU smoke test with a mounted bag.
- `.dockerignore` — keeps `build/ install/ log/` etc. out of the build context.

## Data strategy
- **Model weights are baked in** (`mobileclip2_b.ts`, SAM2, YOLO, spaCy, CLIP) — no
  runtime downloads, reproducible image, works in a locked-down VPC.
- **The rosbag is the variable input** — pulled from S3 at runtime, never baked.
- **API keys are never baked** — inject at runtime from Secrets Manager / SSM.

## Prerequisites (host / EC2)
NVIDIA driver + Docker + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Build
```bash
docker build -t sysnav:latest .
```
(First build is long — it compiles GTSAM/Ceres and CUDA kernels for
detectron2/pytorch3d. These are early, cached layers; source changes only
re-trigger the colcon build.)

## Run locally (mounted bag)
```bash
BAG_HOST=/home/all/AlphaZ/bags/<bag_dir> GEMINI_API_KEY=xxx docker compose up
```

## Run on AWS — EC2 GPU instance (g4dn/g5)
```bash
# push once
aws ecr create-repository --repository-name sysnav
docker tag sysnav:latest <acct>.dkr.ecr.<region>.amazonaws.com/sysnav:latest
docker push <acct>.dkr.ecr.<region>.amazonaws.com/sysnav:latest

# on the GPU instance (IAM role allows s3 + ecr)
docker run --rm --gpus all --network host --ipc host \
  -e BAG_S3_URI=s3://my-bucket/bags/<bag_dir> \
  -e OUTPUT_S3_URI=s3://my-bucket/results/run-001 \
  -e GEMINI_API_KEY="$(aws ssm get-parameter --name /sysnav/gemini_key --with-decryption --query Parameter.Value --output text)" \
  <acct>.dkr.ecr.<region>.amazonaws.com/sysnav:latest
```

## Run on AWS — Batch (one job per bag, recommended for offline processing)
- GPU compute environment (g4dn/g5), job definition points at the ECR image.
- Container args via env: `BAG_S3_URI`, `OUTPUT_S3_URI`.
- Secrets via the job definition `secrets` (Secrets Manager / SSM).
- Job IAM role grants `s3:GetObject`/`s3:PutObject` on the bag + results buckets.
- Each bag = one job; the container exits when the bag finishes and outputs land in S3.

## Known gap
`mission_manager` is **not** in this branch (`az_bags`), so it is commented out
in `run_system.sh`. Uncomment that block once the package is present for a
complete run.

## Slimming later (optional)
`ros-jazzy-desktop-full` is pulled because the rviz/overlay plugins need it *to
build*. A two-stage build (compile with desktop-full, copy `install/` into a
`ros-base` runtime) would cut image size substantially.
