PYTHON = uv run python3
MAIN = fly_in_takeoff.py

run:
	$(PYTHON) $(MAIN)

install:
	uv sync

debug:
	uv run python3 -m pdb $(MAIN)

clean:
	rm -rf __pycache__
	rm -rf .mypy_cache
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

lint:
	flake8 .
	mypy . --warn-return-any \
		--warn-unused-ignores \
		--ignore-missing-imports \
		--disallow-untyped-defs \
		--check-untyped-defs

lint-strict:
	flake8 .
	mypy . --strict

.PHONY: install run debug clean lint lint-strict