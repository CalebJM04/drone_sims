PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
PIO ?= $(if $(wildcard .venv/bin/pio),.venv/bin/pio,pio)
PIO_CORE ?= $(if $(wildcard /tmp/heltec-platformio),/tmp/heltec-platformio,$(CURDIR)/.platformio)
export PYTHONPATH := src

.PHONY: test firmware verify verify-ci quick scenario px4-sitl px4-closed-loop network-matrix mesh-demo mesh-acceptance readiness visualize clean

test:
	$(PYTHON) -m unittest discover -s tests -v

firmware:
	PLATFORMIO_CORE_DIR=$(PIO_CORE) $(PIO) run --project-dir firmware/heltec_bridge

quick:
	$(PYTHON) -m drone_sims verify --cases 1000 --campaign-seeds 3 --endurance-seeds 1 --output results/quick

verify-ci:
	$(PYTHON) -m drone_sims verify --cases 2500 --campaign-seeds 5 --endurance-seeds 1 --workers 2 --output results/ci

verify:
	$(PYTHON) -m drone_sims verify --cases 25000 --campaign-seeds 100 --endurance-seeds 3 --workers 4 --output results/full

scenario:
	$(PYTHON) -m drone_sims scenario head_on --seed 7 --output results/head_on.json

px4-sitl:
	$(PYTHON) tools/run_px4_sitl.py --output results/full/px4_sitl.json

px4-closed-loop:
	$(PYTHON) tools/run_px4_closed_loop.py --output results/full/px4_closed_loop.json

network-matrix:
	$(PYTHON) -m drone_sims network-matrix --seeds 5 --output results/full/network_matrix.json

mesh-demo:
	$(PYTHON) -m drone_sims mesh-demo --nodes 6 --duration 20 --realtime --serve 8080

mesh-acceptance:
	$(PYTHON) -m drone_sims mesh-demo --nodes 6 --duration 15 --backend process

readiness:
	$(PYTHON) tools/build_readiness_report.py

visualize:
	$(PYTHON) -m drone_sims visualize --results results/full --output results/full/dashboard.html

clean:
	find . -type d -name __pycache__ -prune -exec rm -r {} +
