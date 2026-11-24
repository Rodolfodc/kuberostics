# Rendered by infra/modules/workloads (templatefile) and by scripts/check-cloud.sh; one source of truth for kubeconform and Terraform.
# Transport: ${transport} (Q-06, not yet confirmed by the project owner; see infra/README.md).
apiVersion: v1
kind: ConfigMap
metadata:
  name: detector-config
  namespace: ${namespace}
data:
  detector.yaml: |
    detector_cloud:
      ros__parameters:
        model: ${model_file}
%{ if transport == "bridge" ~}
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: bridge-cloud
  namespace: ${namespace}
data:
  bridge.json5: |
    ${indent(4, bridge_config)}
%{ endif ~}
---
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: detector-weights
  namespace: ${namespace}
spec:
  accessModes: [ReadWriteOnce]
  resources:
    requests:
      storage: ${weights_volume_size}
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: detector
  namespace: ${namespace}
  labels: {app: detector}
spec:
  replicas: 1
  strategy:
    type: Recreate   # one Pod owns the weights volume; the breaker covers the gap
  selector:
    matchLabels: {app: detector}
  template:
    metadata:
      labels: {app: detector}
    spec:
      automountServiceAccountToken: false
      securityContext:
        runAsNonRoot: true
        runAsUser: 10001
        fsGroup: 10001
        seccompProfile: {type: RuntimeDefault}
%{ if weights_url != "" ~}
      initContainers:
        - name: fetch-weights   # downloads only when the file is absent; the URL is a variable, weights are never in git or in the image
          image: ${curl_image}
          command: ["sh", "-c", "[ -s \"/weights/$MODEL_FILE\" ] || curl -fsSL -o \"/weights/$MODEL_FILE\" \"$WEIGHTS_URL\""]
          env:
            - {name: MODEL_FILE, value: "${model_file}"}
            - {name: WEIGHTS_URL, value: "${weights_url}"}
          resources:
            requests: {cpu: 50m, memory: 64Mi}
            limits: {cpu: 500m, memory: 256Mi}
          securityContext: {allowPrivilegeEscalation: false, readOnlyRootFilesystem: true, capabilities: {drop: [ALL]}}
          volumeMounts: [{name: weights, mountPath: /weights}]
%{ endif ~}
      containers:
        - name: detector
          image: ${detector_image}
          imagePullPolicy: ${image_pull_policy}
          args: ["ros2", "run", "vacuum_detection", "detector", "--ros-args", "-r", "__node:=detector_cloud", "--params-file", "/config/detector.yaml"]
%{ if transport == "rmw_zenoh" ~}
          env:
            - {name: RMW_IMPLEMENTATION, value: rmw_zenoh_cpp}
            - {name: HOME, value: /tmp}
            - {name: ROS_LOG_DIR, value: /tmp/ros-log}
%{ else ~}
          env:
            - {name: HOME, value: /tmp}
            - {name: ROS_LOG_DIR, value: /tmp/ros-log}
%{ endif ~}
          resources:
            requests: {cpu: "${detector_cpu_request}", memory: ${detector_memory_request}}
            limits: {cpu: "${detector_cpu_limit}", memory: ${detector_memory_limit}}
          securityContext: {allowPrivilegeEscalation: false, readOnlyRootFilesystem: true, capabilities: {drop: [ALL]}}
          livenessProbe:   # node present in the ROS graph (docker/detector/liveness.sh)
            exec: {command: ["/opt/detector/liveness.sh"]}
            initialDelaySeconds: 60
            periodSeconds: 15
            timeoutSeconds: 10
            failureThreshold: 3
          volumeMounts:
            - {name: weights, mountPath: /weights, readOnly: true}
            - {name: config, mountPath: /config, readOnly: true}
            - {name: tmp, mountPath: /tmp}
%{ if transport == "bridge" ~}
        - name: bridge   # shares the pod network namespace with the DDS node (localhost discovery)
          image: ${bridge_image}
          env:
            - {name: ROS_AUTOMATIC_DISCOVERY_RANGE, value: LOCALHOST}
            - {name: ROS_DISTRO, value: ${ros_distro}}
          args: ["-l", "tcp/0.0.0.0:7447", "--no-multicast-scouting", "--rest-http-port", "8000", "-c", "/cfg/bridge.json5"]
          ports: [{containerPort: 7447}]
          resources:
            requests: {cpu: 100m, memory: 128Mi}
            limits: {cpu: "1", memory: 512Mi}
          livenessProbe:
            tcpSocket: {port: 7447}
            initialDelaySeconds: 10
            periodSeconds: 15
          securityContext: {allowPrivilegeEscalation: false, readOnlyRootFilesystem: true, capabilities: {drop: [ALL]}}
          volumeMounts:
            - {name: bridge-config, mountPath: /cfg, readOnly: true}
            - {name: tmp, mountPath: /tmp}
%{ endif ~}
%{ if transport == "rmw_zenoh" ~}
        - name: router   # rmw_zenohd; R19 ACL NOT configured (see infra/README.md), module refuses it unless explicitly allowed
          image: ${detector_image}
          imagePullPolicy: ${image_pull_policy}
          args: ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]
          env:
            - {name: HOME, value: /tmp}
            - {name: ROS_LOG_DIR, value: /tmp/ros-log}
          ports: [{containerPort: 7447}]
          resources:
            requests: {cpu: 100m, memory: 128Mi}
            limits: {cpu: "1", memory: 512Mi}
          livenessProbe:
            tcpSocket: {port: 7447}
            initialDelaySeconds: 15
            periodSeconds: 15
          securityContext: {allowPrivilegeEscalation: false, readOnlyRootFilesystem: true, capabilities: {drop: [ALL]}}
          volumeMounts: [{name: tmp, mountPath: /tmp}]
%{ endif ~}
      volumes:
        - {name: weights, persistentVolumeClaim: {claimName: detector-weights}}
        - {name: config, configMap: {name: detector-config}}
        - {name: tmp, emptyDir: {}}
%{ if transport == "bridge" ~}
        - {name: bridge-config, configMap: {name: bridge-cloud}}
%{ endif ~}
---
apiVersion: v1
kind: Service
metadata:
  name: detector-transport
  namespace: ${namespace}
spec:
  type: NodePort
  selector: {app: detector}
  ports:
    - {name: zenoh, port: 7447, targetPort: 7447, nodePort: ${node_port}}
