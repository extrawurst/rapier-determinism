build-ios:
	cargo lipo --release
	cp target/universal/release/liblogic.a ../_deploy/ios/liblogic.a

build-ios-debug:
	cargo lipo
	cp target/universal/debug/liblogic.a ../_deploy/ios/liblogic.a