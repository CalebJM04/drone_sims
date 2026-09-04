PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
export PYTHONPATH := src

.PHONY: test verify verify-ci quick scenario vectors rtl synth px4-sitl px4-closed-loop network-matrix readiness visualize clean

test:
	$(PYTHON) -m unittest discover -s tests -v

quick:
	$(PYTHON) -m drone_sims verify --cases 1000 --campaign-seeds 3 --endurance-seeds 1 --output results/quick

verify-ci:
	$(PYTHON) -m drone_sims verify --cases 2500 --campaign-seeds 5 --endurance-seeds 1 --workers 2 --output results/ci

verify:
	$(PYTHON) -m drone_sims verify --cases 25000 --campaign-seeds 100 --endurance-seeds 3 --workers 4 --output results/full

scenario:
	$(PYTHON) -m drone_sims scenario head_on --seed 7 --output results/head_on.json

vectors:
	$(PYTHON) -m drone_sims vectors --count 10000 --output results/fpga_golden_vectors.csv

rtl:
	$(PYTHON) tools/run_rtl.py --cases 1000

synth:
	$(PYTHON) tools/run_synthesis.py --output results/full/fpga_synthesis.json

px4-sitl:
	$(PYTHON) tools/run_px4_sitl.py --output results/full/px4_sitl.json

px4-closed-loop:
	$(PYTHON) tools/run_px4_closed_loop.py --output results/full/px4_closed_loop.json

network-matrix:
	$(PYTHON) -m drone_sims network-matrix --seeds 5 --output results/full/network_matrix.json

readiness:
	$(PYTHON) tools/build_readiness_report.py

visualize:
	$(PYTHON) -m drone_sims visualize --results results/full --output results/full/dashboard.html

clean:
	find . -type d -name __pycache__ -prune -exec rm -r {} +
