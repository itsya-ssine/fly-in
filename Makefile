.PHONY: install run debug lint lint-strict clean

MAP = maps/medium/02_circular_loop.txt

PYTHON_FILES = main.py models.py parser.py pathfinder.py simulation.py visualizer.py

install:
	pip install pydantic matplotlib

install-lint:
	pip install flake8 mypy

run:
	python3 main.py $(MAP)

debug:
	python3 -m pdb main.py $(MAP)

lint: install-lint
	flake8 $(PYTHON_FILES)
	mypy $(PYTHON_FILES) --warn-return-any --warn-unused-ignores --ignore-missing-imports \
            --disallow-untyped-defs --check-untyped-defs

lint-strict: install-lint
	flake8 $(PYTHON_FILES)
	mypy $(PYTHON_FILES) --strict

clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name .mypy_cache -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true
