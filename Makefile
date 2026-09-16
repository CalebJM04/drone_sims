PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
export PYTHONPATH := src

.PHONY: test quick verify verify-ci scenario

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
