build_dir := "build"

default:
	@just --list

build:
	cmake -S . -B {{build_dir}} -DBUILD_EXAMPLES=ON -DBUILD_TESTING=ON
	cmake --build {{build_dir}} -j $(nproc)

test:
	cd {{build_dir}} && ctest --output-on-failure

clean:
	find {{build_dir}} -mindepth 1 -delete 2>/dev/null || true

docs-make:
	cd docs && make html

docs-serve:
	cd docs && make serve

docs-clean:
	cd docs && make clean-doc

lint-actions:
	actionlint
