# Convenience entry points for the evaluation harness.
# All artifacts land under outputs/ — see outputs/README.md.

PY := python3
DATASET_ROOT ?= $(HOME)/datasets/wider_face
YUNET_MODEL ?= $(HOME)/models/face_detection/face_detection_yunet_2023mar.onnx
VIDEO_DEMO_SOURCE ?=
VIDEO_DEMO_SAMPLE ?= cdc
VIDEO_DEMO_SECONDS ?= 20
VIDEO_DEMO_START ?= 7
VIDEO_DEMO_RESIZE_WIDTH ?= 960
VIDEO_DEMO_PROCESS_EVERY ?= 1
VIDEO_DEMO_SCORE_THRESHOLD ?= 0.8
VIDEO_DEMO_MATCH_TOLERANCE ?= 0.75
VIDEO_DEMO_IDENTITY_MARGIN ?= 0.15
VIDEO_DEMO_MIN_IDENTITY_FACE_PX ?= 110
VIDEO_DEMO_IDENTITY_SCORE_THRESHOLD ?= 0.90
VIDEO_DEMO_IDENTITY_MAX_HEAD_YAW_DEG ?= 40
VIDEO_DEMO_IDENTITY_MAX_HEAD_PITCH_DEG ?= 25
VIDEO_DEMO_HEADPOSE_SMOOTHING_ALPHA ?= 0.35
VIDEO_DEMO_HEADPOSE_MAX_JUMP_DEG ?= 35
VIDEO_DEMO_HEADPOSE_RESET_AFTER_MISSED ?= 10
VIDEO_DEMO_GAZE_MODEL ?= $(HOME)/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.xml
VIDEO_DEMO_GAZE_DEVICE ?= CPU
VIDEO_DEMO_NO_GAZE ?=
VIDEO_DEMO_OUTPUT ?= $(OUTPUTS_DIR)/runs/video_demo/annotated_engagement_demo.mp4
VIDEO_DEMO_MULTI_START ?= 35
VIDEO_DEMO_MULTI_SECONDS ?= 90
VIDEO_DEMO_MULTI_RESIZE_WIDTH ?= 1920
VIDEO_DEMO_MULTI_PROCESS_EVERY ?= 2
VIDEO_DEMO_MULTI_MIN_IDENTITY_FACE_PX ?= 80
VIDEO_DEMO_MULTI_OUTPUT ?= $(OUTPUTS_DIR)/runs/video_demo/annotated_multi_person_demo.mp4
CHOKEPOINT_GT_OUTPUT ?= $(OUTPUTS_DIR)/runs/chokepoint/chokepoint_gt_face_id_demo.mp4
CHOKEPOINT_GT_SECONDS ?= 30
CHOKEPOINT_GT_MAX_IDENTITIES ?= 12
CHOKEPOINT_GT_VISIBLE_IDENTITIES ?= 6
CHOKEPOINT_GT_IDENTITY_MARGIN ?= 0.15

OUTPUTS_DIR := outputs
BASELINES_DIR := $(OUTPUTS_DIR)/baselines
REPORTS_DIR := $(OUTPUTS_DIR)/reports
FIGURES_DIR := $(OUTPUTS_DIR)/figures
REPORT := $(REPORTS_DIR)/REPORT.md

.PHONY: help
help:
	@echo "Evaluation:"
	@echo "  eval-detection-baseline    dlib HOG on full WIDER FACE val (~10-15 min)"
	@echo "  eval-detection-smoke       dlib HOG on 100 images (~30s)"
	@echo "  eval-detection-yunet       YuNet on full WIDER FACE val (~1.5 min)"
	@echo "  eval-recognition-baseline  dlib 128-D on LFW pairs test (~30s)"
	@echo ""
	@echo "Visualisation:"
	@echo "  visualize                  All figures (detection + recognition + engagement)"
	@echo "  visualize-detection        Detection overlays + AP/latency bars"
	@echo "  visualize-recognition      LFW ROC + similarity distributions"
	@echo "  visualize-engagement       Synthetic engagement timeline"
	@echo "  video-demo                 Annotated demo video under outputs/runs/video_demo/"
	@echo "  video-demo-multi           Longer multi-person annotated demo video"
	@echo "  chokepoint-gt-demo         Ground-truth face/user ID demo video"
	@echo ""
	@echo "Reporting:"
	@echo "  eval-report                outputs/reports/REPORT.md (figures embedded)"
	@echo "  outputs                    eval-* + visualize + eval-report (full pipeline)"
	@echo ""
	@echo "Tests:"
	@echo "  test                       pytest (unit + integration)"
	@echo "  test-unit                  unit tests only"
	@echo ""
	@echo "Cleanup:"
	@echo "  clean-outputs              Wipe all generated files under outputs/"

.PHONY: test
test:
	@$(PY) -m pytest -v

.PHONY: test-unit
test-unit:
	@$(PY) -m pytest test/unit -v

# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------

.PHONY: eval-detection-baseline
eval-detection-baseline:
	@mkdir -p $(BASELINES_DIR)
	$(PY) -m eval.runners.run_detection_eval \
		--backend dlib_hog \
		--dataset-root $(DATASET_ROOT) \
		--output $(BASELINES_DIR)/v0_dlib_hog_wider_val.json \
		--label "v0 dlib HOG baseline (WIDER FACE val, full)"

.PHONY: eval-detection-smoke
eval-detection-smoke:
	@mkdir -p $(BASELINES_DIR)
	$(PY) -m eval.runners.run_detection_eval \
		--backend dlib_hog \
		--dataset-root $(DATASET_ROOT) \
		--output $(BASELINES_DIR)/v0_dlib_hog_smoke_n100.json \
		--limit 100 \
		--label "v0 dlib HOG (smoke, n=100)"

.PHONY: eval-detection-yunet
eval-detection-yunet:
	@mkdir -p $(BASELINES_DIR)
	$(PY) -m eval.runners.run_detection_eval \
		--backend yunet \
		--model $(YUNET_MODEL) \
		--dataset-root $(DATASET_ROOT) \
		--output $(BASELINES_DIR)/v1_yunet_wider_val.json \
		--label "v1 YuNet (WIDER FACE val, full)"

.PHONY: eval-recognition-baseline
eval-recognition-baseline:
	@mkdir -p $(BASELINES_DIR)
	$(PY) -m eval.runners.run_recognition_eval \
		--backend dlib_128d \
		--subset test \
		--output $(BASELINES_DIR)/v0_dlib_128d_lfw.json \
		--label "v0 dlib 128-D (LFW pairs test)"

# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

.PHONY: visualize
visualize: visualize-detection visualize-recognition visualize-engagement

.PHONY: visualize-detection
visualize-detection:
	@mkdir -p $(FIGURES_DIR)/detection
	$(PY) -m eval.visualize detection \
		--baselines-dir $(BASELINES_DIR) \
		--out-dir $(FIGURES_DIR)/detection \
		--dataset-root $(DATASET_ROOT) \
		--yunet-model $(YUNET_MODEL)

.PHONY: visualize-recognition
visualize-recognition:
	@mkdir -p $(FIGURES_DIR)/recognition
	$(PY) -m eval.visualize recognition \
		--out-dir $(FIGURES_DIR)/recognition

.PHONY: visualize-engagement
visualize-engagement:
	@mkdir -p $(FIGURES_DIR)/engagement
	$(PY) -m eval.visualize engagement \
		--out-dir $(FIGURES_DIR)/engagement

.PHONY: video-demo
video-demo:
	@mkdir -p $(OUTPUTS_DIR)/runs/video_demo
	$(PY) -m eval.video_demo \
		$(if $(VIDEO_DEMO_SOURCE),--input $(VIDEO_DEMO_SOURCE),--sample $(VIDEO_DEMO_SAMPLE)) \
		--output $(VIDEO_DEMO_OUTPUT) \
		--max-seconds $(VIDEO_DEMO_SECONDS) \
		--start-seconds $(VIDEO_DEMO_START) \
		--resize-width $(VIDEO_DEMO_RESIZE_WIDTH) \
		--process-every $(VIDEO_DEMO_PROCESS_EVERY) \
		--score-threshold $(VIDEO_DEMO_SCORE_THRESHOLD) \
		--match-tolerance $(VIDEO_DEMO_MATCH_TOLERANCE) \
		--identity-margin $(VIDEO_DEMO_IDENTITY_MARGIN) \
		--min-identity-face-px $(VIDEO_DEMO_MIN_IDENTITY_FACE_PX) \
		--identity-score-threshold $(VIDEO_DEMO_IDENTITY_SCORE_THRESHOLD) \
		--identity-max-head-yaw-deg $(VIDEO_DEMO_IDENTITY_MAX_HEAD_YAW_DEG) \
		--identity-max-head-pitch-deg $(VIDEO_DEMO_IDENTITY_MAX_HEAD_PITCH_DEG) \
		--headpose-smoothing-alpha $(VIDEO_DEMO_HEADPOSE_SMOOTHING_ALPHA) \
		--headpose-max-jump-deg $(VIDEO_DEMO_HEADPOSE_MAX_JUMP_DEG) \
		--headpose-reset-after-missed $(VIDEO_DEMO_HEADPOSE_RESET_AFTER_MISSED) \
		--gaze-model-path $(VIDEO_DEMO_GAZE_MODEL) \
		--gaze-device $(VIDEO_DEMO_GAZE_DEVICE) $(if $(VIDEO_DEMO_NO_GAZE),--no-gaze) \
		--model-path $(YUNET_MODEL)

.PHONY: video-demo-multi
video-demo-multi:
	$(MAKE) video-demo \
		VIDEO_DEMO_SAMPLE=multi_person \
		VIDEO_DEMO_START=$(VIDEO_DEMO_MULTI_START) \
		VIDEO_DEMO_SECONDS=$(VIDEO_DEMO_MULTI_SECONDS) \
		VIDEO_DEMO_RESIZE_WIDTH=$(VIDEO_DEMO_MULTI_RESIZE_WIDTH) \
		VIDEO_DEMO_PROCESS_EVERY=$(VIDEO_DEMO_MULTI_PROCESS_EVERY) \
		VIDEO_DEMO_MIN_IDENTITY_FACE_PX=$(VIDEO_DEMO_MULTI_MIN_IDENTITY_FACE_PX) \
		VIDEO_DEMO_OUTPUT=$(VIDEO_DEMO_MULTI_OUTPUT)

.PHONY: chokepoint-gt-demo
chokepoint-gt-demo:
	@mkdir -p $(OUTPUTS_DIR)/runs/chokepoint
	$(PY) -m eval.chokepoint_demo \
		--output $(CHOKEPOINT_GT_OUTPUT) \
		--seconds $(CHOKEPOINT_GT_SECONDS) \
		--identity-margin $(CHOKEPOINT_GT_IDENTITY_MARGIN) \
		--max-identities $(CHOKEPOINT_GT_MAX_IDENTITIES) \
		--visible-identities $(CHOKEPOINT_GT_VISIBLE_IDENTITIES)

# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

.PHONY: eval-report
eval-report:
	@mkdir -p $(REPORTS_DIR)
	$(PY) -m eval.reports.generate_report \
		$(wildcard $(BASELINES_DIR)/*.json) \
		--output $(REPORT) \
		--figures-dir $(FIGURES_DIR)
	@echo "Wrote $(REPORT)"

.PHONY: outputs
outputs: eval-detection-smoke eval-detection-yunet eval-recognition-baseline \
         visualize eval-report
	@echo ""
	@echo "All outputs ready under $(OUTPUTS_DIR)/"
	@echo "Open $(REPORT) to view the integrated report."

# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------

.PHONY: clean-outputs
clean-outputs:
	@find $(OUTPUTS_DIR) -type f \
		-not -name '.gitignore' \
		-not -name 'README.md' \
		-not -name '.gitkeep' \
		-delete
	@echo "Cleared generated outputs (README/.gitignore/.gitkeep preserved)"
