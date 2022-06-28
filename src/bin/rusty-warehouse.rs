#![allow(non_snake_case)]

use std::fs::OpenOptions;
use std::io::BufWriter;
use hashbrown::HashMap;
use indicatif::{ProgressBar, ProgressStyle};
use rand::rngs::StdRng;
use rand::{SeedableRng, thread_rng};
use rand::prelude::SliceRandom;
use rand::seq::IteratorRandom;
use scpm::algorithm::lp_solver::LPSolver;
use scpm::dfa::definition::{DFA2, Data};
use scpm::agent::{Robot, MDPOps, Agent, serialise_state_mapping, make_serialised_state_map};
use scpm::algorithm::motap_solver::{IMOVISolver, MultiObjSolver};
use scpm::sparse_to_cs;
use scpm::scpm::{definition::{SCPM}, matrix_ops::MatrixOps};
use scpm::scpm::definition::SparseMatrixAttr;
use scpm::solver::*;
use rusty_robots::env::warehouse::low_fidelity_warehouse::{LowResEnv, LowResState, LowResWord};
use rusty_robots::env::warehouse::high_fidelity_warehouse::{create_decoded_sched_to_file, front_pos,
                                                            Info, Point, State, TaskActionPair, warehouse_defaults, WarehouseEnv, WarehouseWord};
use num_cpus;

const THREAD_COUNT_SAVE: usize = 30; // the number of threads to use in threadpools
const THREAD_COUNT_LOAD: usize = 10;

fn main() {

    let cpus_available = num_cpus::get();
    let cpus_used_save: usize = if cpus_available > THREAD_COUNT_SAVE { THREAD_COUNT_SAVE } else { cpus_available };
    let cpus_used_load: usize = if cpus_available > THREAD_COUNT_LOAD { THREAD_COUNT_LOAD } else { cpus_available };
    // Check file system has been created

    let na: usize = 4;
    let nt: usize = 9;

    let w: i32 = 12;
    let h: i32 = 12;
    // ---------------------------------------------------------
    //                Warehouse Setup information
    // ---------------------------------------------------------
    let mut rnd = StdRng::seed_from_u64(1234);
    let feed_points = vec![(0, 5)];
    let queue_points = vec![(11, 11), (0, 11), (3, 11), (9, 0)];
    // randomly choose the feed points for each task
    let task_feed_points = (0..nt)
        .map(|_| *vec![0].choose(&mut rnd).unwrap()).collect::<Vec<usize>>();
    let (mut racks, mut corridors, mut rotation_mapping) =
        warehouse_defaults();
    let mut warehouse_info: Info = Info::make(
        &mut racks,
        &mut corridors,
        &mut rotation_mapping,
        &feed_points[..],
        w as usize,
        h as usize
    );
    warehouse_info.set_racks(None);
    warehouse_info.set_corridors(None);
    warehouse_info.set_rotation_mapping();

    let weight_vector: hashbrown::HashMap<i32, Vec<f64>>;
    let mus: Vec<hashbrown::HashMap<(i32, i32), Vec<f64>>>;
    let mut allocation: hashbrown::HashMap<(usize, usize), usize> = hashbrown::HashMap::new();

    let task_positions: Vec<usize> = warehouse_info.rack_positions
        .iter()
        .enumerate()
        .map(|(i, _p)| i)
        .choose_multiple(&mut rnd, nt);

    let mut agent_start_pos: Vec<Point> = vec![];//(2, 0), (3, 0), (4, 11)];
    for x in 2..w {
        if agent_start_pos.len() < na {
            agent_start_pos.push((x, 0));
            agent_start_pos.push((x, h - 1));
        }
    }

    // Begin first memory block
    {
        let Q = (0..5).collect::<Vec<i32>>();
        let mut low_fidelity_warehouse: Robot<LowResState, LowResWord> =
            Robot::make(4, Default::default());
        low_fidelity_warehouse.state_space(&w, &h, 1);
        low_fidelity_warehouse.transition_map(&1.0, &w, &h, &warehouse_info);

        println!("Agent start positions: {:?}", agent_start_pos);
        println!("warehouse init state: {:?}", low_fidelity_warehouse.get_init_state());
        println!("warehouse init state idx: {:?}", low_fidelity_warehouse.state_mapping.get(&low_fidelity_warehouse.get_init_state()));
        println!("Task positions: {:?}\nTask rack points: {:?}",
                 task_positions,
                 task_positions.iter().map(|&i| &warehouse_info.rack_positions[i]).collect::<Vec<&Point>>()
        );
        println!("Task feeds: {:?}", task_feed_points);
        println!("Queue points: {:?}", queue_points);
        println!("Feed Points: {:?}", feed_points);

        // ------------------------------------------------------------
        //                     Initialise SCPM
        // ------------------------------------------------------------

        let eps: f64 = 0.0001;

        let mut scpm = SCPM::incremental_make(
            low_fidelity_warehouse.num_actions() as i32,
            na,
            nt
        );
        // ------------------------------------------------------------
        // Construct a Threadpool to load transition matrices from disk
        // ------------------------------------------------------------
        let pool = threadpool::ThreadPool::new(cpus_used_load);
        println!("Inputting MDPs");
        let mut initial_robot_states: Vec<LowResState> = vec![Default::default(); na];
        let mut initial_states: HashMap<(i32, i32), usize> = HashMap::new();

        // ------------------------------------------------------------
        //                     Construct Initial States
        // ------------------------------------------------------------
        //scpm.get_switch_indices()
        let bar = ProgressBar::new((na * nt) as u64);
        bar.set_style(ProgressStyle::default_bar()
            .template("[{elapsed_precise}] {bar:40.white/red} {pos:>7}/{len:7} {msg}")
            .progress_chars("##-"));
        for t in 0..nt {
            // generate a bunch of random replenishment tasks
            warehouse_info.lookup_rack = task_positions[t];
            warehouse_info.feed_option = task_feed_points[t];
            for a in 0..na {
                bar.inc(1);
                bar.set_message("constructing initial states");
                low_fidelity_warehouse.init_state = agent_start_pos[a];
                initial_robot_states[a] = low_fidelity_warehouse.init_state.clone();
                let mut task = DFA2::<_, _, &Info>::init(
                    0, &Q, &[3], &[], lr_replenishment, Some(&warehouse_info)
                );
                let mdp = low_fidelity_warehouse.product(&mut task, a as i32, t as i32, Some(&warehouse_info));
                serialise_state_mapping(&make_serialised_state_map(&mdp.reverse_state_mapping), a as i32, t as i32);
                let init_idx = *mdp.state_mapping.get(&mdp.init_state).unwrap();
                initial_states.insert((a as i32, t as i32), init_idx);
            }
        }

        // ---------------------------------------------------------------
        //                     Construct MDP DFA Products
        // ---------------------------------------------------------------

        let bar = ProgressBar::new((na * nt) as u64);
        bar.set_style(ProgressStyle::default_bar()
            .template("[{elapsed_precise}] {bar:40.white/red} {pos:>7}/{len:7} {msg}")
            .progress_chars("##-"));
        // For each task
        for t in 0..nt {
            // generate nt random replenishment tasks
            warehouse_info.lookup_rack = task_positions[t];
            warehouse_info.feed_option = task_feed_points[t];
            // For each agent
            for a in 0..na {
                bar.inc(1);
                bar.set_message("constructing SCPM");
                low_fidelity_warehouse.init_state = agent_start_pos[a];
                let mut task = DFA2::<_,_,&Info>::init(
                    0, &Q, &[3], &[], lr_replenishment, Some(&warehouse_info)
                );
                let mut mdp = low_fidelity_warehouse.product(&mut task, a as i32, t as i32, Some(&warehouse_info));

                let init_idx = *mdp.state_mapping.get(&mdp.init_state).unwrap();
                let next_agent_idx = if a < na - 1 {
                    *initial_states.get(&(a as i32 + 1, t as i32)).unwrap()
                } else {
                    init_idx
                };

                let next_task_idx = if t < nt - 1 {
                    *initial_states.get(&(0, t as i32 + 1)).unwrap()
                } else {
                    init_idx
                };
                scpm.add_mdp_to_self(&mut mdp, next_agent_idx as usize, next_task_idx as usize);
                let act_start = scpm.actions.start;
                let act_end = scpm.actions.end;
                pool.execute(move || {
                    SCPM::incremental_construct_spblas_and_rewards(mdp, act_start, act_end, na, nt);
                });
            }
        }
        pool.join();
        println!("MDP |S|: {:?}, |P|: {:?}", scpm.states, scpm.num_transitions);
        println!("init state: {:?}", scpm.get_init_state(0, 0));
        let mut target = vec![-15.; na];
        let mut ttask = vec![0.99; nt];
        target.append(&mut ttask);

        let (mus_, _hullset, tnew) =
            scpm.imovi_hdd_multi_object_solver(eps, &target[..], 10., 0.1);

        mus = mus_;

        let nacts = low_fidelity_warehouse.action_space().len();

        let mut costs: hashbrown::HashMap<(i32, i32, i32), f64> = hashbrown::HashMap::new();
        let mut probs: hashbrown::HashMap<(i32, i32), f64> = hashbrown::HashMap::new();
        // Compute the task allocation
        for t in 0..nt {
            let mut allocated = vec![0; mus.len()];
            for a in 0..na {
                let init_state = scpm.get_init_state(a as i32, t as i32);
                for k in 0..mus.len() {
                    let sched = mus[k].get(&(a as i32, t as i32)).unwrap();
                    //println!("T: {} => A: {}?, sched: {}, action: {}", t, a, k, sched[*init_state]);
                    if sched[*init_state] != 0. && allocated[k] == 0 {
                        // then load in the matrix and calculate the policy value of the agent
                        // and the probability of completing the task
                        let (c, p) = scpm.runner_policy_value(
                            eps,
                            nacts,
                            &sched[..],
                            a as i32,
                            t as i32
                        );
                        probs.insert((t as i32, k as i32), p);
                        costs.insert((a as i32, t as i32, k as i32), c);
                        allocated[k] = 1;
                        allocation.insert((t, k), a);
                    }
                }
            }
        }

        let solution = scpm.gurobi_task_witness(
            &costs,
            &probs,
            &tnew[..],
            mus.len(),
            nt,
            na)
            .unwrap();

        weight_vector = solution;
        // At this point the SCPM, low fidelity warehouse, and anything else that is in the memory
        // block has been dropped
    }

    // construct a task runner function which loads in the required matrices based on the allocation
    // function

    println!("Making high fidelity warehouse");
    // construct a high fidelity warehouse
    let mut high_fidelity_warehouse: Robot<State, WarehouseWord> =
        Robot::warehouse_make(5, Default::default());
    high_fidelity_warehouse.warehouse_state_space(&warehouse_info.corridor_positions[..]);
    high_fidelity_warehouse.warehouse_transition_map(&1., &warehouse_info);

    // construct the mdp of the task to the allocated agent

    let mut pi_mappings: std::collections::HashMap<(i32, i32), std::collections::HashMap<String, std::collections::HashMap<String, Vec<TaskActionPair>>>> =
        std::collections::HashMap::new();
    let mut agent_costs: Vec<f64> = vec![0.; na];
    let mut allocations_per_agent: Vec<Vec<usize>> = vec![Vec::new(); na];

    for t in 0..nt {
        let k = weight_vector.get(&(t as i32))
            .unwrap()
            .iter()
            .enumerate()
            .map(|(i, x)| (i, *x)).collect::<Vec<(usize, f64)>>()
            .choose_weighted(&mut thread_rng(), |elem| elem.1).unwrap().0;
        let agent = allocation.get(&(t, k)).unwrap();
        warehouse_info.lookup_rack = task_positions[t];
        warehouse_info.feed_option = task_feed_points[t];

        high_fidelity_warehouse.init_state = State {
            agent_dir: 1,
            agent_position: agent_start_pos[*agent].clone(),
            carrying: 0,
            pack_available: 0,
            pack_position: (-1, -1)
        };
        println!(
            "task: {} => {:?}, k = {}, agent allocated => {}",
            t,
            weight_vector.get(&(t as i32)),
            k,
            agent
        );
        allocations_per_agent[*agent].push(t);

        let Q = (0..8).collect::<Vec<i32>>();
        // construct the DFA
        let mut task = DFA2::<_,_,&Info>::init(
            0, &Q, &[5], &[7],
            hr_replenishment, Some(&warehouse_info)
        );
        // construct the Product MDP
        println!("making product {} x {}", agent, t);
        let mut mdp = high_fidelity_warehouse.product(
            &mut task,
            *agent as i32,
            t as i32,
            Some(&warehouse_info)
        );

        // Get the initial state of the product DFA
        let init_idx = *mdp.state_mapping.get(&mdp.init_state).unwrap();

        //println!("Executing planning for product");
        // Construct all of the necessary data structures to conduct sparse value iteration on the MDP
        //
        let eps: f64 = 1e-5;
        // determine the set of proper policies and randomly choose an initial one
        let ns: usize = mdp.states.len();

        let proper_policies = proper_policies(&mut mdp);
        // determine the available actions for each state
        let available_actions = set_available_actions(
            &mut mdp,
            high_fidelity_warehouse.action_space().start,
            high_fidelity_warehouse.action_space().end
        );
        mdp_rewards_fn(
            &mut mdp,
            high_fidelity_warehouse.action_space().start,
            high_fidelity_warehouse.action_space().end
        );

        let mut cs_matricies: Vec<_> = Vec::new();
        let mut rewards_map: HashMap<i32, Vec<f64>> = HashMap::new();

        let mdp_reverse_state_mapping: HashMap<usize, (i32, i32)>;

        {
            // compute the sparse matricies involved; Transitions and Rewards for each action
            // the mdp get consumed at this step and therefore we need to save the reverse state map to
            // memory before we consume the MDP
            mdp_reverse_state_mapping = mdp.reverse_state_mapping.clone();

            let (transition_matrices, mut rewards_matrices) = construct_spblas_and_rewards(
                mdp,
                high_fidelity_warehouse.action_space().start,
                high_fidelity_warehouse.action_space().end,
            );
            for action in high_fidelity_warehouse.action_space().start..high_fidelity_warehouse.action_space().end {
                let S = transition_matrices.get(&action).unwrap();
                cs_matricies.push(SparseMatrixAttr {
                    m: sparse_to_cs(S),
                    nr: S.nr as usize,
                    nc: S.nc as usize,
                    nnz: S.nz as usize
                });
                rewards_map.insert(action, rewards_matrices.remove(&action).unwrap());
            }
        }

        let (pi, objvals) = mdp_sparse_value_iter(
            eps,
            high_fidelity_warehouse.action_space().len(),
            high_fidelity_warehouse.actions.start,
            high_fidelity_warehouse.actions.end,
            ns,
            init_idx,
            &proper_policies,
            &available_actions,
            &cs_matricies[..],
            &rewards_map
        );
        // Construct a file of the scheduler
        //println!("obj vals: {:?}", objvals);

        pi_mappings.insert((*agent as i32, t as i32), create_decoded_sched_to_file(
            &pi[..],
            &mdp_reverse_state_mapping,
            &high_fidelity_warehouse.reverse_state_mapping
        ));
        agent_costs[*agent] += objvals[0];
    }
    let pool = threadpool::ThreadPool::new(cpus_used_save);

    for ((a, t), v) in pi_mappings.into_iter() {
        pool.execute(move || {
            let pth = format!("{}/schedulers/", std::env::var("SCPM_HOME").unwrap());
            let filename = format!("{}/map_{}_{}.txt", pth, a, t);
            let file = OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open(filename).unwrap();
            let f = BufWriter::new(file);
            serde_json::to_writer_pretty(
                f,
                &v
            ).unwrap();
        });
    }
    pool.join();


    // ------------------------------------------------------
    //                Regeneration Schedulers
    // ------------------------------------------------------
    // Now we need a scheduler which returns the agents back to the queue position
    let mut regeneration_schedulers: std::collections::HashMap<usize, std::collections::HashMap<String, std::collections::HashMap<String, Vec<TaskActionPair>>>> =
        std::collections::HashMap::new();
    // For each agent compute a scheduler which gets to the queue position for this agent
    for a in 0..na {
        high_fidelity_warehouse.init_state = State {
            agent_dir: 1,
            agent_position: agent_start_pos[a].clone(),
            carrying: 0,
            pack_available: 0,
            pack_position: (-1, -1)
        };
        warehouse_info.queue_point = queue_points[a].clone();
        let Q = (0..3).collect::<Vec<i32>>();
        // construct the DFA
        let mut task = DFA2::<_,_,&Info>::init(
            0, &Q, &[1], &[],
            regeneration, Some(&warehouse_info)
        );
        // construct the Product MDP
        println!("making regeneration task");
        let mut mdp = high_fidelity_warehouse.product(
            &mut task,
            a as i32,
            0,
            Some(&warehouse_info)
        );

        // Get the initial state of the product DFA
        let init_idx = *mdp.state_mapping.get(&mdp.init_state).unwrap();

        //println!("Executing planning for product");
        // Construct all of the necessary data structures to conduct sparse value iteration on the MDP
        //
        let eps: f64 = 1e-5;
        // determine the set of proper policies and randomly choose an initial one
        let ns: usize = mdp.states.len();

        let proper_policies = proper_policies(&mut mdp);
        // determine the available actions for each state
        let available_actions = set_available_actions(
            &mut mdp,
            high_fidelity_warehouse.action_space().start,
            high_fidelity_warehouse.action_space().end
        );
        mdp_rewards_fn(
            &mut mdp,
            high_fidelity_warehouse.action_space().start,
            high_fidelity_warehouse.action_space().end
        );

        let mut cs_matricies: Vec<_> = Vec::new();
        let mut rewards_map: HashMap<i32, Vec<f64>> = HashMap::new();

        let mdp_reverse_state_mapping: HashMap<usize, (i32, i32)>;

        {
            // compute the sparse matricies involved; Transitions and Rewards for each action
            // the mdp get consumed at this step and therefore we need to save the reverse state map to
            // memory before we consume the MDP
            mdp_reverse_state_mapping = mdp.reverse_state_mapping.clone();

            let (transition_matrices, mut rewards_matrices) = construct_spblas_and_rewards(
                mdp,
                high_fidelity_warehouse.action_space().start,
                high_fidelity_warehouse.action_space().end,
            );
            for action in high_fidelity_warehouse.action_space().start..high_fidelity_warehouse.action_space().end {
                let S = transition_matrices.get(&action).unwrap();
                cs_matricies.push(SparseMatrixAttr {
                    m: sparse_to_cs(S),
                    nr: S.nr as usize,
                    nc: S.nc as usize,
                    nnz: S.nz as usize
                });
                rewards_map.insert(action, rewards_matrices.remove(&action).unwrap());
            }
        }

        let (pi, _objvals) = mdp_sparse_value_iter(
            eps,
            high_fidelity_warehouse.action_space().len(),
            high_fidelity_warehouse.actions.start,
            high_fidelity_warehouse.actions.end,
            ns,
            init_idx,
            &proper_policies,
            &available_actions,
            &cs_matricies[..],
            &rewards_map
        );

        regeneration_schedulers.insert(a, create_decoded_sched_to_file(
            &pi[..],
            &mdp_reverse_state_mapping,
            &high_fidelity_warehouse.reverse_state_mapping
        ));
    }
    let pool = threadpool::ThreadPool::new(30);

    for (a, v) in regeneration_schedulers.into_iter() {
        pool.execute(move || {
            let pth = format!("{}/schedulers/", std::env::var("SCPM_HOME").unwrap());
            let filename = format!("{}/regen_{}.txt", pth, a);
            let file = OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open(filename).unwrap();
            let f = BufWriter::new(file);
            serde_json::to_writer_pretty(
                f,
                &v
            ).unwrap();
        });
    }
    pool.join();
}

fn goto_rack_position(data: &Data<LowResWord, &Info>, qprime: i32, q: i32) -> i32 {
    let info_ref = data.info.as_ref().unwrap();
    if data.w.agent_position == info_ref.rack_positions[info_ref.lookup_rack] {
        return qprime
    } else {
        return q
    }
}

fn goto_feed_position(data: &Data<LowResWord, &Info>) -> i32 {
    let info_ref = data.info.as_ref().unwrap();
    if data.w.agent_position == info_ref.feed_points[info_ref.feed_option] {
        return 2
    } else {
        return 1
    }
}

fn finish(q: i32) -> i32 {
    q
}

fn done(q: i32) -> i32 {
    q
}

fn lr_replenishment(data: &Data<LowResWord, &Info>) -> i32 {
    let qprime = match data.q {
        0 => {Ok(goto_rack_position(&data, 0, 1))}
        1 => {Ok(goto_feed_position(&data))}
        2 => {Ok(goto_rack_position(&data, 3, 2))}
        3 => {Ok(finish(4))}
        4 => {Ok(done(4))}
        _ => {Err("Q state not found")}
    };
    match qprime {
        Ok(i) => { i }
        Err(_) => { -1 }
    }
}

/// DFA transition telling the robot to goto some random rack position
fn goto_rand_rackpos(data: &Data<WarehouseWord, &Info>) -> i32 {
    let info_ref = data.info.as_ref().unwrap();
    let front = front_pos(
        &data.w.agent_position,
        &data.w.dir,
        info_ref.rotation_mapping,
        info_ref.width,
        info_ref.height
    );
    if *data.w.is_carrying() == 0 {
        match front {
            Some(point) => {
                if point == info_ref.rack_positions[info_ref.lookup_rack] {
                    return 1
                }
            }
            None => { }
        }
    } else {
        return 7
    }
    return 0
}

fn pickup_rack_at_pos(data: &Data<WarehouseWord, &Info>) -> i32 {
    if *data.w.is_carrying() == 1 {
        return 2
    } else {
        return 1
    }
}

fn carry_rack_to_feed0(data: &Data<WarehouseWord, &Info>) -> i32 {
    let info_ref = data.info.as_ref().unwrap();
    let front = front_pos(
        &data.w.agent_position,
        &data.w.dir,
        info_ref.rotation_mapping,
        info_ref.width,
        info_ref.height
    );
    if *data.w.is_carrying() == 1 {
        match front {
            Some(point) => {
                if point == info_ref.feed_points[info_ref.feed_option] {
                    return 3
                }
            }
            None => { }
        }
    } else {
        return 7
    }
    return 2
}

// return the rack back to its original position
fn carry_rack_back_to_pos(data: &Data<WarehouseWord, &Info>) -> i32 {
    let info_ref = data.info.as_ref().unwrap();
    let front = front_pos(
        &data.w.agent_position,
        &data.w.dir,
        info_ref.rotation_mapping,
        info_ref.width,
        info_ref.height
    );
    if *data.w.is_carrying() == 1 {
        match front {
            Some(point) => {
                if point == info_ref.rack_positions[info_ref.lookup_rack] {
                    return 4
                }
            }
            None => { }
        }
    } else {
        return 7
    }
    return 3
}

fn drop_rack(data: &Data<WarehouseWord, &Info>) -> i32 {
    if *data.w.is_carrying() == 0 {
        return 5
    } else {
        return 4
    }
}

fn fail() -> i32 {
    7
}

fn hr_replenishment(data: &Data<WarehouseWord, &Info>) -> i32 {
    //let qfail = 6;
    let qprime = match data.q {
        0 => {Ok(goto_rand_rackpos(&data))}
        1 => {Ok(pickup_rack_at_pos(&data))}
        2 => {Ok(carry_rack_to_feed0(&data))}
        3 => {Ok(carry_rack_back_to_pos(&data))}
        4 => {Ok(drop_rack(&data))}
        5 => {Ok(finish(6))}
        6 => {Ok(done(6))}
        7 => {Ok(fail())}
        _ => {Err("Q state not found")}
    };
    match qprime {
        Ok(i) =>  { i }
        Err(_) => { -1 }
    }
}

fn goto_queue_pos(data: &Data<WarehouseWord, &Info>) -> i32 {
    let info_ref: &&Info = data.info.as_ref().unwrap();
    if data.w.agent_position == info_ref.queue_point {
        return 1
    }
    return 0
}

fn finish_regen() -> i32 {
    2
}

fn done_regen() -> i32 {
    2
}

fn regeneration(data: &Data<WarehouseWord, &Info>) -> i32 {
    let qprime = match data.q {
        0 => { Ok(goto_queue_pos(&data)) }
        1 => {Ok(finish_regen())}
        2 => {Ok(done_regen())}
        _ => { Err("Q state not found")}
    };
    match qprime {
        Ok(i) => { i }
        Err(_) => { -1 }
    }
}