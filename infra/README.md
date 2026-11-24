# infra: cloud side on the local K3s cluster

Terraform for the local k3d cluster `kuberostics`. Specs: 009a (offload, R17, R19) and the cloud cycle of `docs/architecture.md`
section 4. Run only through the wrappers: `tools/k3s.sh` (cluster) and `tools/tf.sh` (fixed to this directory and to the
project's own kubeconfig). `apply`, `destroy` and `delete` need the user's confirmation.

## Layout

| Path | What |
|---|---|
| `main.tf`, `variables.tf` | root: `kubernetes` provider (no `config_path`: `tools/tf.sh` exports `KUBE_CONFIG_PATH` and `KUBE_CTX`) |
| `modules/workloads` | namespace `vacuum-cloud`, detector Deployment (`livenessProbe`, weights volume, ConfigMaps, transport Service), optional cloud-sync scaffold |
| `../deploy/k8s/*.yaml.tpl` | the manifests, as templates; Terraform and `scripts/check-cloud.sh` render the same files |
| `../deploy/bridge/*.json5` | `zenoh-bridge-ros2dds` allow-lists (R19): `robot.json5` and `bridge-cloud.json5` |
| `../docker/detector` | detector image, `ARG ROS_DISTRO` |

## Detector image: built locally, never on GitHub Actions

The image is about 5 GB (CPU torch, ultralytics, ROS). CI (`.github/workflows/cloud.yaml`) only validates Terraform and manifests.
Build it by hand, one distro at a time (Art. X), from the repository root:

```bash
docker build -f docker/detector/Dockerfile --build-arg ROS_DISTRO=jazzy  -t vacuum-detector:jazzy  .
docker build -f docker/detector/Dockerfile --build-arg ROS_DISTRO=lyrical -t vacuum-detector:lyrical .
```

## Transport (Q-06): an assumption

The project owner has **not** confirmed Q-06. `variable "transport"` is the switch:

- `bridge` (default): `zenoh-bridge-ros2dds` sidecar with the R19 allow-list (only `/myRobot/perception/rgb_throttled/compressed` in and
  `/myRobot/perception/detections/cloud` out). NodePort 30448. This is the recommendation of `deploy/measurements/robot-cloud-transport-benchmark.md`.
- `rmw_zenoh`: `rmw_zenohd` router in the Pod, NodePort 30447. It has **no ACL here**, so it does not meet R19. The module refuses it
  (precondition) unless `-var allow_unfiltered_rmw_zenoh=true`, which is for measurements only. Writing the ACL and re-examining Art. VI/X
  are part of the owner's decision.

The robot side of the bridge is `compose.yaml` (service `bridge`, profile `offload`): `BRIDGE_CLOUD_ENDPOINT=tcp/<k3d node IP>:30448`.

## Usage

```bash
scripts/check-cloud.sh                       # Docker only: fmt, validate, render, policy checks, kubeconform (no cluster)
tools/tf.sh init -backend=false && tools/tf.sh validate     # offline verbs, no cluster needed
tools/k3s.sh up                              # creates the local cluster (state change: asks for confirmation)
docker build -f docker/detector/Dockerfile --build-arg ROS_DISTRO=jazzy -t vacuum-detector:jazzy .
k3d image import -c kuberostics vacuum-detector:jazzy      # the Pod never pulls the image (imagePullPolicy: Never)
tools/tf.sh plan -var ros_distro=jazzy       # read-only against the cluster
tools/tf.sh apply -var ros_distro=jazzy      # ASKS for confirmation; never run it unattended
```

State (`*.tfstate*`), `.terraform/` and `.k3s/kubeconfig` stay local and are gitignored. `.terraform.lock.hcl` is committed
(`scripts/check-cloud.sh --update-lock` regenerates it).

## Weights and secrets (none in git, none in images)

- **YOLO weights**: never committed (`*.pt`, `*.onnx` are gitignored) and not in the image. The Pod mounts the PVC `detector-weights` at
  `/weights`. Set `-var weights_url=<https url>`: an init container downloads the file once when it is missing (the weights mount is read-only in the detector container, so `kubectl cp` into it does not work; a one-off helper Pod mounting the same PVC is the alternative). The URL must not carry credentials.
- **MQTT and the weights store** (`-var enable_cloud_sync=true`) read credentials from Secrets that you create out of band; the values
  never enter Terraform or git:

```bash
tools/k3s.sh -n vacuum-cloud create secret generic policy-store-credentials \
  --from-literal=root-user=<user> --from-literal=root-password=<generated password>
mosquitto_passwd -c -b passwd <user> <password>          # in a container: eclipse-mosquitto:2.0.20
tools/k3s.sh -n vacuum-cloud create secret generic mosquitto-passwd --from-file=passwd=./passwd
```

## Detector Pod assumptions (to confirm with the detector owner)

- Executable `ros2 run vacuum_detection detector` (T906, not yet in the repo), node name `detector_cloud`, parameter `model`
  (a weights path under `/weights`). `model` is not in `contracts/topics.yaml` yet; the owner of the contract should add it.
- `livenessProbe` is `docker/detector/liveness.sh`: the node is in the ROS graph. A heartbeat from the detector would be a better probe.
- Until T906 lands the image builds and the Pod would crash-loop: that is expected, not a deployment defect.

## Not built (blocked)

Ingestion, training Job, digital twin, dashboard, DynamoDB Local, trap generator, LLM agent and the `aws` module are not scaffolded:
spec 009b does not exist, and training depends on spec 008 and on D-11 (steps per second, not settled for in-sim RL). Only the broker
(Mosquitto) and the weights store (MinIO, versioned bucket to be created with the `mc` client by the cloud cycle) are declared, behind `enable_cloud_sync`.
