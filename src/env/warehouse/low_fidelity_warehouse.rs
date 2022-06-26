use std::collections::HashMap;
use std::fs::OpenOptions;
use itertools::iproduct;
use scpm::agent::{Agent, MDPState, Robot};
use scpm::scpm::definition::TaskAgentStateActionPair;
use serde::Serialize;
use crate::env::warehouse::high_fidelity_warehouse::{Info, Point};

pub type LowResState = (i32, i32);

#[derive(Clone)]
pub struct LowResWord {
    pub agent_position: Point
}

impl LowResWord {
    /*fn get_agent_position(&self) -> &Point {
        &self.agent_position
    }*/

    fn new(p: &Point) -> Self {
        LowResWord {
            agent_position: *p
        }
    }
}

impl Default for LowResWord {
    fn default() -> Self {
        LowResWord{
            agent_position: (0, 0)
        }
    }
}

pub trait LowResEnv<T, S, W> where T: Agent<S, W>, W: Clone {
    fn make(na: i32, init_state: S) -> T;

    fn state_space(&mut self, w: &i32, h: &i32, grid_square: usize) -> (i32, i32);

    fn step(&mut self, state: &S, a: i32, w: &i32, h: &i32, info: &Info)
        -> Result<Vec<(i32, f64, W)>, &'static str>;

    fn transition_map(&mut self, r: &f64, w: &i32, h: &i32, info: &Info);
}

impl LowResEnv<Robot<LowResState, LowResWord>, LowResState, LowResWord> for Robot<LowResState, LowResWord> {
    fn make(na: i32, init_state: LowResState) -> Robot<LowResState, LowResWord> {
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

    /// The state space is a low resolution version of the the full grid map representation of a
    /// warehouse
    /// w: is the original warehouse width
    /// h: is the original warehouse height
    /// grid_square is the low resolution mapping of original grid squares, for example 4 original
    /// squares fit into 1 low res square -> grid_square = 4
    ///
    /// Returns the grid size of the new low res grid
    fn state_space(&mut self, w: &i32, h: &i32, grid_square: usize) -> (i32, i32) {
        let wnew = ((*w as f64 ) / (grid_square as f64)).ceil() as i32;
        let hnew = ((*h as f64 ) / ( grid_square  as f64)).ceil() as i32;
        for (ix, (x, y)) in iproduct!((0..wnew), (0..hnew)).enumerate() {
            self.set_state(&(x, y));
            self.insert_state_mapping(&(x, y), ix);
            self.insert_word(LowResWord {
                agent_position: (x, y)
            });
        }
        self.set_reverse_state_mapping();
        (wnew, hnew)
    }

    fn step(&mut self, state: &LowResState, a: i32, w: &i32, h: &i32, info: &Info)
        -> Result<Vec<(i32, f64, LowResWord)>, &'static str> {

        let (_min_x, min_y) = info.rack_positions.iter().min().unwrap();
        let (max_x, max_y) = info.rack_positions.iter().max().unwrap();

        let (px, py) = state;
        // Left
        let (xnew, ynew) = if a == 0 {
            if *px > 0 {
                (*px - 1, *py)
            } else {
                (*px, *py)
            }
        } else if a == 1 {
            // Right
            if *px < *w - 1 {
                (*px + 1, *py)
            } else {
                (*px, *py)
            }
        } else if a == 2 {
            // up
            //if *px >= *min_x + 2 && *px <= *max_x - 2 && *py >= *min_y &&*py <= *max_y && *px % 2 != 0 {
            if *px <= *max_x - 2 && *py >= *min_y &&*py <= *max_y && *px % 2 != 0 {
                (*px, *py)
            } else {
                if *py < *h - 1 {
                    (*px, *py + 1)
                } else {
                    (*px, *py)
                }
            }
        } else if a == 3 {
            // down
            //if *px >= *min_x + 2 && *px <= *max_x - 2 && *py >= *min_y &&*py <= *max_y && *px % 2 != 0 {
            if *px <= *max_x - 2 && *py >= *min_y &&*py <= *max_y && *px % 2 != 0 {
                (*px, *py)
            } else {
                if *py > 0 {
                    (*px, *py - 1)
                } else {
                    (*px, *py)
                }
            }
        } else {
            panic!("Action not found")
        };
        let snew: LowResState = (xnew, ynew);
        let sidx = self.get_state_mapping().get(&snew).unwrap();
        Ok(vec![(*sidx as i32, 1.0, LowResWord::new(&snew))])
    }

    fn transition_map(&mut self, r: &f64, w: &i32, h: &i32, info: &Info) {
        let state_space = self.states.to_vec();
        for s in state_space.iter() {
            let sidx = *self.get_state_mapping().get(s).unwrap();
            for a in self.actions.start..self.actions.end {
                match self.step(s, a, w, h, info) {
                    Ok(v) => {
                        self.transitions.insert((sidx as i32, a), v);
                        self.insert_reward(sidx as i32, a, *r);
                    }
                    Err(e) => {
                        panic!("{:?}", e)
                    }
                }
            }
        }
    }
}


#[derive(Serialize, Debug)]
struct MapResult {
    a: i32,
    state: LowResState
}

pub struct Mappings {
    pub current: HashMap<usize, MDPState>,
    pub next_agent: Option<HashMap<usize, MDPState>>,
    pub next_task: Option<HashMap<usize, MDPState>>
}


#[derive(Serialize)]
struct Mapping {
    state: LowResState,
    value: MapResult
}

pub fn construct_agent_scheduler_hdd<'a, W> (
    robot: &Robot<LowResState, W>,
    mappings: &Mappings,
    data: &[&TaskAgentStateActionPair],
    state_mapping_fname: &str,
    robot_initial_states: &[LowResState]) {
    let mut new_map: Vec<Mapping> = Vec::new();
    // load in the state mapping for the agent
    let pth = format!("{}/schedulers", std::env::var("SCPM_HOME").unwrap());
    let filename = format!("{}/{}", pth, state_mapping_fname);
    for state_action in data.iter() {
        let sq = mappings.current.get(&state_action.s).unwrap();
        let sqprime = if state_action.sprime_a == state_action.s_a + 1 && state_action.sprime_t == state_action.s_t {
            mappings.next_agent.as_ref().unwrap().get(&state_action.sprime).unwrap()
        } else if state_action.sprime_a == 0 && state_action.sprime_t == state_action.s_t + 1 {
            mappings.next_task.as_ref().unwrap().get(&state_action.sprime).unwrap()
        } else {
            mappings.current.get(&state_action.sprime).unwrap()
        };
        //let sqprime = mdp.reverse_state_mapping.get(&state_action.sprime).unwrap();
        println!("agent: {}, task: {}, sidx: {}, s: {:?}, a: {} => sidx': {}, s': {:?}, a: {} t: {}",
                 state_action.s_a, state_action.s_t, state_action.s, sq, state_action.action, state_action.sprime,
                 sqprime, state_action.sprime_a, state_action.sprime_t);
        let s = robot.reverse_state_mapping.get(&(sq.s as usize)).unwrap().clone();
        let sprime = if state_action.sprime_a != state_action.s_a || state_action.sprime_t != state_action.s_t {
            robot_initial_states[state_action.sprime_a].clone()
        } else {
            robot.reverse_state_mapping.get(&(sqprime.s as usize)).unwrap().clone()
        };
        new_map.push(
            Mapping {
                state: s,
                value: MapResult {
                    a: state_action.action,
                    state: sprime
                }
            }
        );
    }
    let file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(filename).unwrap();
    serde_json::to_writer_pretty(
        &file,
        &new_map
    ).unwrap();
}