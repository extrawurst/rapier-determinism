build-ios:
	cargo lipo --release
	cp target/universal/release/liblogic.a ../_deploy/ios/liblogic.a

# 
clean-web:
	rm target/wasm32-unknown-emscripten/release/logic.wasm || true
	rm target/wasm32-unknown-emscripten/release/deps/logic.wasm || true
	rm target/wasm32-unknown-emscripten/release/deps/logic.wasm.map || true

# single command to quick iteration
rebuild-web: clean-web build-web

build-web:
	RUSTFLAGS="-C link-args=-fPIC -C relocation-model=pic -C target-feature=+mutable-globals" \
	EMMAKEN_CFLAGS="-O1 -s STRICT=1 -s SIDE_MODULE=1" \
	C_INCLUDE_PATH="$(EMSDK)/upstream/emscripten/cache/sysroot/include/" \
	EMCC_STRICT=1 \
	CARGO_PROFILE_RELEASE_PANIC=abort \
	CARGO_TARGET_WASM32_UNKNOWN_EMSCRIPTEN_LINKER="./emcc.sh" \
	cargo b --release --target wasm32-unknown-emscripten
	exa -la target/wasm32-unknown-emscripten/release/logic.wasm
	wasm-validate target/wasm32-unknown-emscripten/release/logic.wasm