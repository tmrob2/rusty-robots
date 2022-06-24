#![allow(dead_code)]

use itertools::{Itertools};
use scpm::agent::{Agent, Robot};
use hashbrown::HashMap;
use serde::{Serialize, Deserialize};
use crate::env::gym_env::Env;

pub type Point = (i32, i32);

/*
Directions:
1 = right
2 = Down
3 = Left
4 = Up
*/

#[derive(Serialize, Deserialize)]
pub struct TaskActionPair {
    pub agent_dir: u8, // {1: Right, 2: Down, 3: Left, 4: Up}
    pub agent_position: Point, // any position on the available grid (x, y)
    pub carrying: u8, // {0, 1}
    pub pack_available: u8, // {0, 1}
    pub pack_position: Point,
    pub action: i32,
    pub q: i32
}

pub enum CellType {
    OutOfBounds,
    Free,
    Pack,
    Rack,
    Feed
}

#[derive(Hash, Eq, PartialEq, Copy, Clone, Debug, Serialize, Deserialize)]
pub struct State {
    pub agent_dir: u8, // {1: Right, 2: Down, 3: Left, 4: Up}
    pub agent_position: Point, // any position on the available grid (x, y)
    pub carrying: u8, // {0, 1}
    pub pack_available: u8, // {0, 1}
    pub pack_position: Point // any position on the available grid
    // that is not the agent position, (-1, -1) means is the default value
    // and we look at the pack available first
}

impl Default for State {
    fn default() -> Self {
        State {
            agent_dir: 0,
            agent_position: (1, 0),
            carrying: 0,
            pack_available: 0,
            pack_position: (-1, -1)
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct WarehouseWord {
    pub agent_position: Point,
    pub dir: u8,
    pub carrying: u8,
    pub pack_position: Option<Point>
}

impl WarehouseWord {
    pub fn get_agent_position(&self) -> &Point {
        &self.agent_position
    }

    pub fn is_carrying(&self) -> &u8 {
        &self.carrying
    }

    pub fn new(pos: Point, dir: u8, carrying: u8, packpos: Option<Point>) -> Self {
        WarehouseWord {
            agent_position: pos,
            dir,
            carrying,
            pack_position: packpos
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct RobotMetaData {
    pub feed_points: Vec<Point>,
    pub width: usize,
    pub height: usize,
}

pub struct Info<'a> {
    pub rack_positions: &'a mut Vec<Point>,
    pub lookup_rack: usize,
    pub feed_option: usize,
    pub corridor_positions: &'a mut Vec<Point>,
    pub feed_points: &'a [Point],
    pub width: usize,
    pub height: usize,
    pub rotation_mapping: &'a mut HashMap<u8, (i32, i32)>
}

impl<'a> Info<'a> {
    pub fn make(
        racks: &'a mut Vec<Point>,
        corridors: &'a mut Vec<Point>,
        rotation_mapping: &'a mut HashMap<u8, (i32, i32)>,
        feedpoints: &'a [Point],
        w: usize,
        h: usize
    ) -> Info<'a> {
        Info {
            rack_positions: racks,
            lookup_rack: 0,
            feed_option: 0,
            corridor_positions: corridors,
            feed_points: feedpoints,
            width: w,
            height: h,
            rotation_mapping
        }
    }
    
    pub fn set_racks(&mut self, input: Option<Vec<Point>>) {
        match input {
            Some(mut x) => {
                self.rack_positions.append(&mut x);
            }
            None => {
                let cells = (self.width - 2) / 3;
                // Check that cells >= 1
                assert!(cells >= 1, "The warehouse is not wide enough to fit \
                any racks, make the width dimension larger or check a \
                horizontal configuration"); // if this check fails then construction of the warehouse will fail
                for c in 0..cells {
                    for y in 1..self.height - 1 {
                        // add the racks
                        for ii in 0..2 {
                            self.rack_positions.push(((c * 3 + 2 + ii) as i32, y as i32));
                        }
                    }
                }
            }
        }
    }
    
    pub fn set_corridors(&mut self, input: Option<Vec<Point>>) {
        match input {
            Some(mut x) => {
                self.corridor_positions.append(&mut x)
            }
            None => {
                let gx: Vec<i32> = (0..self.width as i32).collect();
                let gy: Vec<i32> = (0..self.height as i32).collect();
                let rack_positions = self.rack_positions.to_vec();
                let feed = self.feed_points.clone();
                for (x, y) in gx.into_iter()
                    .cartesian_product(gy.into_iter())
                    .filter(|grid: &Point|
                        !(rack_positions.iter().any(|p| grid == p) && feed.iter().any(|p| grid == p))
                    ) {
                    self.corridor_positions.push((x as i32, y as i32));
                }
            }
        }
    }

    pub fn set_lookup_rack(&mut self, rack: usize) {
        self.lookup_rack = rack;
    }

    pub fn get_lookup_rack(&self) -> &usize {
        &self.lookup_rack
    }
    
    pub fn set_rotation_mapping(&mut self) {
        self.rotation_mapping.insert(0, (1, 0)); // right
        self.rotation_mapping.insert(1, (0, 1)); // down
        self.rotation_mapping.insert(2, (-1, 0)); // left
        self.rotation_mapping.insert(3, (0, -1)); // up
    }
}

pub fn warehouse_defaults() -> (Vec<Point>, Vec<Point>, HashMap<u8, (i32, i32)>) {
    let rotation_mapping: HashMap<u8, (i32, i32)> = HashMap::new();
    let rack_positions: Vec<Point> = Vec::new();
    let corridor_positions: Vec<Point> = Vec::new();
    (rack_positions, corridor_positions, rotation_mapping)
}

pub fn front_pos(
    agent_position: &Point,
    dir: &u8,
    rotation_mapping: &HashMap<u8, (i32, i32)>,
    w: usize,
    h: usize
) -> Option<Point> {
    // rx is the x rotation
    // ry is the y rotation
    let (ax, ay) = agent_position;
    let (rx, ry) = match rotation_mapping.get(dir) {
        Some(p) => *p,
        None => {panic!("dir: {}, not found", dir)}
    };
    let x = ax + rx;
    let y = ay + ry;
    if x >= 0 && y >= 0 && x < w as i32 && y < h as i32 {
        Some((x, y))
    } else {
        None
    }
}

pub fn fwd_cell(
    p: Option<Point>,
    pack_available: u8,
    pack_point: &Point,
    rack_positions: &[Point],
    feedpoint: &[Point]
) -> CellType {
    match p {
        None => { CellType::OutOfBounds }
        Some(point) => {
            // does the agent face a pack, rack, or free position
            if rack_positions.iter().any(|x| *x == point) {
                CellType::Rack
            } else if pack_available == 1 && *pack_point == point {
                CellType::Pack
            } else if feedpoint.iter().any(|&p| p == point) {
                CellType::Feed
            } else {
                CellType::Free
            }
        }
    }
}

impl Env<Robot<State, WarehouseWord>, State, WarehouseWord> for Robot<State, WarehouseWord> {
    fn make(na: i32, init_state: State) -> Robot<State, WarehouseWord> {
        Robot {
            states: vec![],
            init_state,
            actions: 0..na,
            labels: Default::default(),
            transitions: Default::default(),
            rewards: Default::default(),
            alphabet: vec![],
            state_mapping: Default::default(),
            reverse_state_mapping: Default::default()
        }
    }

    fn state_space(&mut self) {
        // we can't use this state space generation becuase it won't have all of the information
        // that is required. Therefore we will need to use a super trait and calll that from the
        // binary. The super trait is defined below
    }

    fn step(&mut self, _state: &State, _a: i32) -> Result<Vec<(State, f64, WarehouseWord)>, &'static str> {
        Ok(vec![])
    }

    fn transition_map(&mut self, _r: &f64) { }
}

pub trait WarehouseEnv: Env<Robot<State, WarehouseWord>, State, WarehouseWord> + Agent<State, WarehouseWord> {
    fn warehouse_make(na: i32, init_state: State) -> Robot<State, WarehouseWord> {
        Self::make(na, init_state)
    }

    fn warehouse_state_space(
        &mut self,
        corridor_positions: &[Point],
    ) {
        let mut state_counter: usize = 0;
        for p in corridor_positions.iter() {
            for dir in 0..4u8 {
                // this is all of the positions that the agent could occupy
                // place the agent in one of these positions
                // now that this position has been taken up by the agent
                // - the agent may or may not be carrying something
                for c in 0..2u8 {
                    match c {
                        0 => {
                            // the agent is not carrying anything and therefore
                            // it is possible that a pack could be placed somewhere
                            // on the grid
                            // get the remaining positions
                            let remaining_corridor_positions =
                                corridor_positions.iter().filter(|p2| *p2 != p);
                            for p2 in remaining_corridor_positions {
                                let mut state: State = Default::default();
                                state.agent_position = *p;
                                state.carrying = 0;
                                state.pack_available = 1;
                                state.pack_position = *p2;
                                state.agent_dir = dir;
                                self.set_state(&state);
                                self.insert_state_mapping(&state, state_counter);
                                self.insert_word(WarehouseWord::new(
                                    state.agent_position, 
                                    state.agent_dir, 
                                    state.carrying,
                                    Some(state.pack_position) 
                                ));
                                state_counter += 1;
                            }
                            // Initial situations where the agent is not carrying a pack and no pack has
                            // been placed on the grid
                            let mut state: State = Default::default();
                            state.agent_position = *p;
                            state.carrying = 0;
                            state.agent_dir = dir;
                            self.set_state(&state);
                            self.insert_state_mapping(&state, state_counter);
                            self.insert_word(WarehouseWord::new(
                                state.agent_position, 
                                state.agent_dir, 
                                state.carrying, 
                                None
                            ));
                            state_counter += 1;
                        }
                        1 => {
                            // the agent is carrying something, and we assume that
                            // the agent has not yet dropped the item somewhere
                            // on the grid; meaning that there is no pack
                            let mut state: State = Default::default();
                            state.agent_position = *p;
                            state.agent_dir = dir;
                            state.carrying = 1;
                            self.set_state(&state);
                            self.insert_state_mapping(&state, state_counter);
                            self.insert_word(WarehouseWord::new(
                                state.agent_position, 
                                state.agent_dir, 
                                state.carrying,
                                None
                            ));
                            state_counter += 1;
                        }
                        _ => {
                            // this should not be possible, we could throw an error
                            // but it does not need to be handled
                        }
                    }
                }
            }
        }
        self.set_reverse_state_mapping();
    }

    fn warehouse_step(&self, state: &State, a: i32, info: &Info)
        -> Result<Vec<(State, f64, WarehouseWord)>, &'static str> {
        let mut new_dir = state.agent_dir;
        let mut new_agent_positions: Point = state.agent_position;
        let mut new_carrying = state.carrying;
        let mut is_pack = state.pack_available;
        let mut pack_position = state.pack_position;
        let fwd_position = front_pos(
            &state.agent_position,
            &state.agent_dir, info.rotation_mapping, info.width, info.height
        );
        let fwd_cell = fwd_cell(
            fwd_position,
            is_pack,
            &pack_position,
            &info.rack_positions[..],
            &info.feed_points
        );
        match a {
            // rotate left
            0 => {
                if new_dir == 0 {
                    new_dir += 3
                } else {
                    new_dir -= 1;
                }
                //new_dir -= 1;
                //if new_dir < 0 {
                //    new_dir += 4;
                //}
                //println!("new dir: {:?}", new_dir);
            }
            // rotate right
            1 => {
                new_dir = (new_dir + 1) % 4;
            }
            // go forward
            2 => {
                match fwd_cell {
                    CellType::Free => {
                        // go forward
                        new_agent_positions = fwd_position.unwrap();
                    }
                    _ => { }
                }
            }
            // pickup
            3 => {
                if state.carrying == 0 {
                    match fwd_cell {
                        CellType::Feed => {
                            // only pick up from feed if there is currenlty not a pack on the floor
                            if state.pack_available == 0 {
                                new_carrying = 1;
                            }
                        }
                        CellType::Pack => {
                            new_carrying = 1;
                            is_pack = 0;
                            pack_position = (-1, - 1);
                        }
                        CellType::Rack => {
                            // only pick up from the rack if there is a no pack on the floor
                            if state.pack_available == 0 {
                                new_carrying = 1;
                            }
                        }
                        _ => { }
                    }
                }
            }
            // place
            4 => {
                if state.carrying == 1 {
                    match fwd_cell {
                        CellType::Rack => {
                            new_carrying = 0;
                            pack_position = (-1, -1);
                            is_pack = 0;
                        }
                        CellType::Feed => {
                            new_carrying = 0;
                            pack_position = (-1, -1);
                            is_pack = 0;
                        }
                        CellType::Free => {
                            new_carrying = 0;
                            pack_position = fwd_position.unwrap();
                            is_pack = 1;
                        }
                        _ => { }
                    }
                }
            }
            _ => {
                // error
            }
        }
        let mut new_state: State = Default::default();
        new_state.agent_position = new_agent_positions;
        new_state.carrying = new_carrying;
        new_state.agent_dir = new_dir;
        new_state.pack_available = is_pack;
        new_state.pack_position = pack_position;

        let warehouse_word = WarehouseWord {
            agent_position: new_agent_positions,
            dir: new_dir,
            carrying: new_carrying,
            pack_position: if new_state.pack_available == 1 { Some(new_state.pack_position) } else { None }
        };
        Ok(vec![(new_state, 1.0, warehouse_word)])
    }

    fn warehouse_transition_map(&mut self, r: &f64, info: &Info) {
        let states = self.get_states().to_vec();
        for state in states.iter() {
            let state_idx = *self.get_state_mapping().get(state).unwrap() as i32;
            for a in 0..self.action_space().end {
                let sprimes = self.warehouse_step(state, a, info).unwrap();
                let sprime_mapping: Vec<(i32, f64, WarehouseWord)> = sprimes
                    .iter()
                    .map(|(s, p, w)|
                        (*self.get_state_mapping().get(s).unwrap() as i32, *p, w.clone())
                    )
                    .collect();
                self.insert_transition(state_idx, a, sprime_mapping);
                self.insert_reward(state_idx, a, *r)
            }
        }
    }
}

pub fn create_decoded_sched_to_file(
    pi: &[f64],
    reverse_state_map: &HashMap<usize, (i32, i32)>,
    robot_reverse_state_map: &HashMap<usize, State>
) -> std::collections::HashMap<String, std::collections::HashMap<String, Vec<TaskActionPair>>>{
    // for each state in the policy
    //
    // get the product mdp state which will always take the form: (i32, i32)
    //
    // with the first member of the tuple, convert this to a usize,
    // then convert the usize to a robot state
    // construct a tree structure to save
    // states should be grouped by position and then by agent direction
    let mut sched_fn: std::collections::HashMap<String, std::collections::HashMap<String, Vec<TaskActionPair>>> =
        std::collections::HashMap::new();
    for sidx in 0..pi.len() {
        let (s, q) = reverse_state_map.get(&sidx).unwrap();
        let robot_state = robot_reverse_state_map.get(&(*s as usize)).unwrap();
        // write the robot state and the action taken at the robot state
        match sched_fn.get_mut(format!("{:?}", robot_state.agent_position).as_str()) {
            None => {
                let mut new_dir_pair: std::collections::HashMap<String, Vec<TaskActionPair>> =
                    std::collections::HashMap::new();
                new_dir_pair.insert(format!("{}", robot_state.agent_dir), vec![
                    TaskActionPair {
                        agent_dir: robot_state.agent_dir,
                        agent_position: robot_state.agent_position,
                        carrying: robot_state.carrying,
                        pack_available: robot_state.pack_available,
                        pack_position: robot_state.pack_position,
                        action: pi[sidx] as i32,
                        q: *q
                    }
                ]);
                sched_fn.insert(format!("{:?}", robot_state.agent_position), new_dir_pair);
            }
            Some(dirmap) => {
                match dirmap.get_mut(format!("{:?}", robot_state.agent_dir).as_str()) {
                    None => {
                        dirmap.insert(format!("{}", robot_state.agent_dir), vec![
                            TaskActionPair {
                                agent_dir: robot_state.agent_dir,
                                agent_position: robot_state.agent_position,
                                carrying: robot_state.carrying,
                                pack_available: robot_state.pack_available,
                                pack_position: robot_state.pack_position,
                                action: pi[sidx] as i32,
                                q: *q
                            }
                        ]);
                    }
                    Some(v) => {
                        v.push(
                            TaskActionPair {
                                agent_dir: robot_state.agent_dir,
                                agent_position: robot_state.agent_position,
                                carrying: robot_state.carrying,
                                pack_available: robot_state.pack_available,
                                pack_position: robot_state.pack_position,
                                action: pi[sidx] as i32,
                                q: *q
                            }
                        );
                    }
                }
            }
        }
    }
    sched_fn
}

impl WarehouseEnv for Robot<State, WarehouseWord> { }
