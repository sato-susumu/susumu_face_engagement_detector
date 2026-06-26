# Convenience entry points for the evaluation harness.
# All artifacts land under outputs/ — see outputs/README.md.

PY := python3
DATASET_ROOT ?= $(HOME)/datasets/wider_face
YUNET_MODEL ?= $(HOME)/models/face_detection/face_detection_yunet_2023mar.onnx

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
