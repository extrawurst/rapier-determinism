use crate::utils::NodeExt;
use crc::crc16;
use gdnative::api::Engine;
use gdnative::prelude::*;
use rapier2d::{
    dynamics::{
        CCDSolver, IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    },
    geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase},
    na,
    pipeline::PhysicsPipeline,
};
use std::cell::RefCell;

#[derive(NativeClass)]
#[inherit(Node2D)]
pub struct RapierWorld2D {
    #[property]
    pub gravity: Vector2,

    ticks: u64,
    hash: u16,
    pipeline: RefCell<PhysicsPipeline>,
    broad_phase: RefCell<BroadPhase>,
    narrow_phase: RefCell<NarrowPhase>,
    bodies: RefCell<RigidBodySet>,
    colliders: RefCell<ColliderSet>,
    joints: RefCell<JointSet>,
    ccd: RefCell<CCDSolver>,
    boxes: RefCell<Vec<(RigidBodyHandle, Ref<Node2D>)>>,
}

#[methods]
impl RapierWorld2D {
    fn new(_owner: &Node2D) -> Self {
        Self {
            ticks: 0,
            hash: 0,
            gravity: Vector2::new(0., 98.),
            pipeline: RefCell::new(PhysicsPipeline::new()),
            broad_phase: RefCell::new(BroadPhase::new()),
            narrow_phase: RefCell::new(NarrowPhase::new()),
            bodies: RefCell::new(RigidBodySet::new()),
            colliders: RefCell::new(ColliderSet::new()),
            joints: RefCell::new(JointSet::new()),
            boxes: RefCell::new(Vec::new()),
            ccd: RefCell::new(CCDSolver::new()),
        }
    }

    #[export]
    fn _ready(&self, owner: &Node2D) {
        let w = owner.get_viewport_rect().width();
        let h = owner.get_viewport_rect().height();

        godot_print!("size: {},{}", w, h);

        self.add_static(0., h, w, 10.);
        self.add_static(0., h / 2., 10., h);
        self.add_static(w, h / 2., 10., h);
    }

    #[export]
    fn _process(&mut self, owner: &Node2D, _delta: f64) {
        let label: TRef<Label> = owner.get_typed_node("../LabelFps");
        label.set_text(format!(
            "FPS: {}",
            Engine::godot_singleton().get_frames_per_second()
        ));

        let label: TRef<Label> = owner.get_typed_node("../LabelTick");
        label.set_text(format!("Tick: {}", self.ticks));
        let label: TRef<Label> = owner.get_typed_node("../LabelHash");
        label.set_text(format!("Hash: {}", self.hash));

        self.tick_rapier(owner);
        self.update_hash();
        self.update_boxes(owner);
    }

    fn tick_rapier(&mut self, owner: &Node2D) {
        if self.ticks < 1000 {
            self.ticks += 1;

            if self.ticks % 2 == 0 {
                self.spawn(owner, 400., 100.);
            }

            let mut pipeline = self.pipeline.borrow_mut();
            let mut broad_phase = self.broad_phase.borrow_mut();
            let mut narrow_phase = self.narrow_phase.borrow_mut();
            let mut bodies = self.bodies.borrow_mut();
            let mut colliders = self.colliders.borrow_mut();
            let mut joints = self.joints.borrow_mut();
            let mut ccd = self.ccd.borrow_mut();

            let gravity = na::Vector2::new(self.gravity.x, self.gravity.y);
            let mut integration_parameters = IntegrationParameters::default();
            integration_parameters.max_position_iterations = 1;
            integration_parameters.max_linear_correction = 1.;

            pipeline.step(
                &gravity,
                &integration_parameters,
                &mut broad_phase,
                &mut narrow_phase,
                &mut bodies,
                &mut colliders,
                &mut joints,
                &mut ccd,
                &(),
                &(),
            );
        }
    }

    fn update_hash(&mut self) {
        // let broad_phase = self.broad_phase.borrow().clone();
        let narrow_phase = self.narrow_phase.borrow().clone();
        let bodies = self.bodies.borrow().clone();
        let colliders = self.colliders.borrow().clone();
        let joints = self.joints.borrow().clone();

        let mut strings: Vec<String> = Vec::new();
        strings.push(serde_json::to_string(&bodies).unwrap());
        strings.push(serde_json::to_string(&colliders).unwrap());
        // strings.push(serde_json::to_string(&broad_phase).unwrap());
        strings.push(serde_json::to_string(&narrow_phase).unwrap());
        strings.push(serde_json::to_string(&joints).unwrap());

        self.hash = crc16::checksum_x25(strings.join(";").as_bytes());
    }

    fn update_boxes(&self, _owner: &Node2D) {
        let bodies = self.bodies.borrow_mut();

        for b in self.boxes.borrow().iter() {
            let handle = b.0;
            let node = b.1;
            let body = bodies.get(handle).unwrap();
            let pos = body.position();

            let node = unsafe { node.assume_safe() };

            node.set_position(Vector2::new(pos.translation.x, pos.translation.y));
            node.set_rotation(pos.rotation.angle() as f64);
        }
    }

    fn spawn(&self, owner: &Node2D, x: f32, y: f32) {
        let mut bodies = self.bodies.borrow_mut();
        let mut colliders = self.colliders.borrow_mut();
        let mut boxes = self.boxes.borrow_mut();

        let falling_box = RigidBodyBuilder::new_dynamic().translation(x, y).build();

        let falling_box = bodies.insert(falling_box);
        let falling_box_index = boxes.len();

        let box_collider = ColliderBuilder::cuboid(48. * 0.4, 48. * 0.4).build();
        colliders.insert(box_collider, falling_box, &mut bodies);

        {
            let box_asset = load_scene("res://scenes/Box.tscn").unwrap();

            let new_node = instance_scene::<Node2D>(&box_asset);

            let key_str = format!("box_{}", falling_box_index);
            new_node.set_name(&key_str);

            let shared_node = new_node.into_shared();
            owner.add_child(shared_node, false);

            boxes.push((falling_box, shared_node));
        }
    }

    fn add_static(&self, x: f32, y: f32, w: f32, h: f32) {
        let floor = RigidBodyBuilder::new_static().translation(x, y).build();

        let mut bodies = self.bodies.borrow_mut();
        let floor = bodies.insert(floor);

        let floor_collider = ColliderBuilder::cuboid(w, h).build();

        let mut colliders = self.colliders.borrow_mut();

        colliders.insert(floor_collider, floor, &mut bodies);
    }
}

pub fn load_scene(path: &str) -> Option<Ref<PackedScene, ThreadLocal>> {
    let scene = ResourceLoader::godot_singleton().load(path, "PackedScene", false)?;

    let scene = unsafe { scene.assume_thread_local() };

    scene.cast::<PackedScene>()
}

fn instance_scene<Root>(scene: &PackedScene) -> Ref<Root, Unique>
where
    Root: gdnative::GodotObject<RefKind = ManuallyManaged> + SubClass<Node>,
{
    let instance = scene
        .instance(PackedScene::GEN_EDIT_STATE_DISABLED)
        .unwrap();
    let instance = unsafe { instance.assume_unique() };

    instance.try_cast::<Root>().unwrap()
}
