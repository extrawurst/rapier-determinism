mod rapier_world;
mod sound;
mod utils;

use gdnative::prelude::*;
use sound::run_example;

fn init(handle: InitHandle) {
    godot_print!("gdnative init");
    run_example();
    handle.add_class::<rapier_world::RapierWorld2D>();
}

fn shutdown(info: &gdnative::TerminateInfo) {
    godot_print!("shutdown: (in_editor: {})", info.in_editor());
}

godot_gdnative_init!();
godot_nativescript_init!(init);
godot_gdnative_terminate!(shutdown);
