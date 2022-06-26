<div id="top"></div>

<!-- PROJECT SHIELDS -->
<!--[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Apache-2.0 License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]
-->
[![Issues][issues-shield]][issues-url]
[![Apache License][license-shield]][license-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
<h3 align="center">Rusty Robots</h3>

  <p align="center">
    A rust implementation for solving multiple-objective task allocation problems in multiagent systems 
    <br />
    <a href="https://github.com/tmrob2/rusty-robots"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    ·
    <a href="https://github.com/tmrob2/rusty-robots/issues">Report Bug</a>
    ·
    <a href="https://github.com/tmrob2/rusty-robots/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<summary>Table of Contents</summary>
<ol>
<li>
  <a href="#about-the-project">About The Project</a>
  <ul>
    <li><a href="#built-with">Built With</a></li>
  </ul>
</li>
<li>
  <a href="#getting-started">Getting Started</a>
  <ul>
    <li><a href="#prerequisites">Prerequisites</a></li>
    <li><a href="#installation">Installation</a></li>
  </ul>
</li>
<li><a href="#usage">Usage</a></li>
<li><a href="#roadmap">Roadmap</a></li>
<li><a href="#contributing">Contributing</a></li>
<li><a href="#license">License</a></li>
<li><a href="#contact">Contact</a></li>
<li><a href="#acknowledgments">Acknowledgments</a></li>
</ol>



<!-- ABOUT THE PROJECT -->
## About The Project

Allocating a set of tasks to a set of agents is an NP-hard problem. To determine the
cost that an agent expects that a task will take usually some form of rich model is 
required. Especially when there is uncertainty in robot action outcomes and multiple objectives are required. 
This prototype project seeks to define the robot environments, in the form of Markov decision processes (MDP), 
and tasks, in the form of deterministic finite automata (DFA), which can be applied to them. 
It then calls a multiobjective task allocation and planning (MOTAP) software using a model called a 
sequential composition product MDP (SCPM),
which efficiently computes the task allocation and a set of schedulers which agents can execute 
concurrently. An SCPM consists of virtual transitions which link product MDPs for efficient computation purposes.

The following graphic demonstrates the allocation of tasks to a set of agents in a warehouse simulation as
a high level demonstration of the MOTAP capabilities. In this diagram, 9 tasks are allocated to 4 agents 
while satisfying the multiobjective tradeoff between the cost required for an agent to execute a set of tasks,
and the probability requirement of completing of tasks. The agents adhere to traffic management rules, and some
simple collision avoidance rules are implemented. This demonstrates that although task allocation and planning are carried
out according to the conditions of the SCPM, powerful multiagent models can still be generated. This is a relatively small
example, and much larger systems can be easily generated. 


[![Product Name Screen Shot][product-screenshot]](https://github/tmrob2/rusty-robots/)

<p align="right">(<a href="#top">back to top</a>)</p>



### Built With

* [Rust v1.61](https://www.rust-lang.org/)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

### Why Rust?
Rust is a fast memory safe language which is very useful in this scenario. In models with large state-spaces
we will want to construct a model, do some computation, and then drop it without memory leaks. We can guarantee
memory safety using this framework. This project also uses state of the art linear algebra libraries to compute
matrix-vector products building upon 20 years of sparse-matrix computation rather than re-implementing these
libraries. 

### Project Setup
Clone project and compile with Cargo using the associated manifest. 
```shell
cargo build --bin=env_bin --release
cargo run --bin=env_bin --release
```

On compilation of this crate, a file structure will be created that will be used to store data when constructing
an SCPM. The file structure looks like:

An environment variable `SCPM_HOME` is required to specify where this file structure should be located. It is best to 
add this to the bottom of your `.bashrc`. 

### Prerequisites

To construct an agent environment and solve a MOTAP problem we require `motap-hdd` 
from https://github.com/tmrob2/motap-hdd. 
Follow the setup instructions of this project. 
This will require also setting up `Gurobi Optimizer` (for solving Linear Programs) and `CXSparse` (as a
sparse matrix solver).

Cargo - The Rust package manager.

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

### Constructing a Model of an Environment

This project is designed as a scalable API for configuring large task allocation problems. First we
require some environment. An environment can be constructed as follows. To implement an environment we have 
to implement the Env trait `env::gym_env::Env`. Environments can be added to the
`env` directory.
```rust
pub trait Env<T, S, W> where T: Agent<S, W>, W: Clone {
    // Construct the env and store in mem
    fn make(na: i32, init_state: S) -> T;
    // Construct a state space for the environment, states are generic S typed and can therefore be anything
    // as long as the satisfy the trait requirements of the SCPM
    fn state_space(&mut self);
    // A transition step function
    fn step(&mut self, state: &S, a: i32) -> Result<Vec<(S, f64, W)>, &'static str>;
    // A transition map working through each state in the state space and applying a transition function 
    // and word
    fn transition_map(&mut self, r: &f64);
}
```
See a full example

<details>
<summary>Environment Example</summary>

#### Message Sending Example

[![Product Name Screen Shot][mdp-example]](https://github/tmrob2/rusty-robots/)

```rust
use hashbrown::HashMap;
use crate::mdp::agent::{Robot};
use std::iter::FromIterator;
use crate::envs::gym_env::Env;

type State = i32;
pub type XpState = (i32, i32);
pub type XpSprimePr = (XpState, f64);

impl Env<Robot<State, &'static str>, State, &'static str> for Robot<State, &'static str> {
    fn make(na: i32, init_state: i32) -> Self {
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
        self.states = (0i32..=4).collect();
        self.state_mapping = HashMap::from_iter(
            self.states.iter().enumerate().map(|(ix, s)| (*s, ix))
        );
        self.reverse_state_mapping = HashMap::from_iter(
            self.states.iter().enumerate().map(|(ix, x)| (ix, *x))
        );
    }

    fn step(&mut self, state: &i32, a: i32) -> Result<Vec<(i32, f64, &'static str)>, &'static str> {
        if *state == 0 {
            match a {
                0 => { Ok(vec![(0, 0.01, "begin"), (1, 0.99, "init")]) }
                _ => { Err("Action must be 0") }
            }
        } else if *state == 1 {
            match a {
                0 => { Ok(vec![(2, 1.0, "ready")]) }
                _ => { Err("Action must be 0") }
            }
        } else if *state == 2 {
            match a {
                0 => { Ok(vec![(4, 0.01, "exit"), (3, 0.99, "send")]) }
                1 => { Ok(vec![(4, 1.0, "exit")]) }
                _ => { Err("Action must be 0 or 1") }
            }
        } else if *state == 3 {
            match a {
                0 => { Ok(vec![(2, 1.0, "ready")]) }
                _ => { Err("Action must be 0") }
            }
        } else {
            match a {
                0 => { Ok(vec![(0, 1.0, "begin")]) }
                _ => { Err("Action must be 0") }
            }
        }
    }

    fn transition_map(&mut self, r: &f64) {
        let state_space = self.states.to_vec();
        for s in state_space.iter() {
            for a in self.actions.start..self.actions.end {
                match self.step(s, a) {
                    Ok(v) => {
                        self.transitions.insert((*s, a), v);
                        self.rewards.insert((*s, a), *r);
                    }
                    Err(_) => {}
                }
            }
        }
    }
}
```

</details>

### Constructing a Task
To specify DFA the following convention can be followed. Suppose that we want to verify that a robot goes to 
a certain position and is facing a specific direction. In this framework a DFA is comprised of two generics
`W` which is a word and `T` which is usually a pointer to data that the DFA needs to know about the environment.
We can specify a deterministic transition function using the following:
```rust
// LowResWord corresponds to a word in an alphabet in this case
// Info is a memory reference to all information about the environment not needing to be cloned
fn goto_rack_position(data: &Data<LowResWord, &Info>, qprime: i32, q: i32) -> i32 {
    let info_ref = data.info.as_ref().unwrap();
    if data.w.agent_position == info_ref.rack_positions[info_ref.lookup_rack] {
        return qprime
    } else {
        return q
    }
}
```
The function specifies what to do at state `q` in a DFA. If an agent is at a particular position
then we move forward in the DFA `q'` otherwise we stay at the current state `q`.

A task can then be formed by specifying all states and which function to implement at this particular state.
The following is an example of a replenishment task in a warehouse. At each state in the DFA a transition
function is implemented to determine which state to move to next. 
```rust
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
```


### Constructing a Rewards Model

The rewards model for an environment gets computed on every step in the transition map. 
The following example implements a constant reward `r` for reward function `R(s, a)` for each 
state and action.

```rust
fn transition_map(&mut self, r: &f64) {
    let state_space = self.states.to_vec();
    for s in state_space.iter() {
        for a in self.actions.start..self.actions.end {
            match self.step(s, a) {
                Ok(v) => {
                    self.transitions.insert((*s, a), v);
                    self.rewards.insert((*s, a), *r);
                }
                Err(_) => {}
            }
        }
    }
}
```

### Forming a Product MDP 

A product MDP is a tuple $$ \mathcal{M} (S \times Q, (s_0, q_0), A, P', L') $$ for each
$i$-agent, $j$-task where,
* S - MDP state space
* Q - DFA state space
* $(s_0, q_0)$ - initial state
* $A$ - set of actions 
* $L$ - Labelling function $L': S \times Q \mapsto 2^\Sigma$
* $P'$ - transition function $P': (S \times Q)  \times A \times (S \times Q ) \mapsto [0, 1]$

The product MDP is implemented in the `motap-hdd` crate and can be called by defining a task
and calling product from an environment reference. For example a warehouse product MDP could be constructed
with something like the following:
```rust
let Q = (0..5).collect::<Vec<i32>>()
// The DFA signature via init is:
// (initial, state_space, accepting_states, rejecting_states, task_fn, extra_data)
let mut task = DFA2::<_, _, &Info>::init(0, &Q, &[3], &[], lr_replenishment, Some(&warehouse_info)); 
let mut mdp = low_fidelity_warehouse.product(&mut task, a as i32, t as i32, Some(&warehouse_info));
```

Usually these product MDPs are very demanding on memory, and as this framework aims for scalability
we have to save the MDP state mapping $(S, Q) \mapsto \mathtt{usize}$.

```rust
serialise_state_mapping(&make_serialised_state_map(&mdp.reverse_state_mapping), a as i32, t as i32);
```

Also the transition matrices must be saved to disk as they quickly overload system resources. This is 
handled on construction of the SCPM and gets saved to the `transitions` directory. 
### Constructing an SCPM

An SCPM is constructed incrementally by constructing each of the $\mathcal{M_i} \times \mathcal{A_j}$, and 
labelling functions, saving them to disk, and then dropping the product from memory. This is implemented
with a memory block. 

#### Setting the Initial States
For incremental construction of the SCPM, the initial states are first defined and stored in a `HashMap`
otherwise we will have no idea where our virtual transitions connecting the product MDPs in the SCPM will 
lead to. Taking the warehouse example this looks like the following: 
```rust
// ---------------------------------------------------------
//                     Construct Initial States
// ---------------------------------------------------------
// incorporate a progress bar as this is a long running computation
// and we want to keep updated on our progress
let bar = ProgressBar::new((na * nt) as u64);
bar.set_style(ProgressStyle::default_bar()
    .template("[{elapsed_precise}] {bar:40.white/red} {pos:>7}/{len:7} {msg}")
    .progress_chars("##-"));
// Loop over all of the tasks
for t in 0..nt {
    // generate a bunch of random replenishment tasks
    warehouse_info.lookup_rack = task_positions[t];
    warehouse_info.feed_option = task_feed_points[t];
    // loop over all of the agents 
    for a in 0..na {
        // increment the progress bar by one
        bar.inc(1);
        bar.set_message("constructing initial states");
        // set the regeneration point of the warehouse so that an agent may efficiently compute a 
        // number of consecutive tasks
        low_fidelity_warehouse.init_state = agent_start_pos[a];
        initial_robot_states[a] = low_fidelity_warehouse.init_state.clone();
        // Construct the task DFA
        let mut task = DFA2::<_, _, &Info>::init(
            0, &Q, &[3], &[], lr_replenishment, Some(&warehouse_info)
        );
        // construct the product MDP
        let mut mdp = low_fidelity_warehouse.product(&mut task, a as i32, t as i32, Some(&warehouse_info));
        // save the state mappings to disk, we will use this later to get the inverse mapping from the 
        // SCPM state to the MDP state back to the environment state
        serialise_state_mapping(&make_serialised_state_map(&mdp.reverse_state_mapping), a as i32, t as i32);
        // set the initial state of the product MDP
        let init_idx = *mdp.state_mapping.get(&mdp.init_state).unwrap();
        // capture the inital state in a HashMap
        initial_states.insert((a as i32, t as i32), init_idx);
    }
}
```

#### Incrementally Construct the SCPM

Once we have found the initial states, we can go about constructing the SCPM. The following snippet follows
our running warehouse example:
```rust
// ---------------------------------------------------------
//                     Construct Products
// ---------------------------------------------------------
// construct a progress bar as this is a long running task
let bar = ProgressBar::new((na * nt) as u64);
bar.set_style(ProgressStyle::default_bar()
    .template("[{elapsed_precise}] {bar:40.white/red} {pos:>7}/{len:7} {msg}")
    .progress_chars("##-"));
// loop over all of the tasks
for t in 0..nt {
    // generate nt random replenishment tasks
    warehouse_info.lookup_rack = task_positions[t];
    warehouse_info.feed_option = task_feed_points[t];
    // loop over all of the agents
    for a in 0..na {
        // increment the progress bar
        bar.inc(1);
        bar.set_message("constructing SCPM");
        low_fidelity_warehouse.init_state = agent_start_pos[a];
        // construct the task - bit of duplication but because we are memory contrained 
        // we can't re-use a reference
        let mut task = DFA2::<_,_,&Info>::init(
            0, &Q, &[3], &[], lr_replenishment, Some(&warehouse_info)
        );
        // construct the DP product
        let mut mdp = low_fidelity_warehouse.product(&mut task, a as i32, t as i32, Some(&warehouse_info));
        // get the initial state
        let init_idx = *mdp.state_mapping.get(&mdp.init_state).unwrap();
        // Set the next agent M x A init state [For switch transition]
        let next_agent_idx = if a < na - 1 {
            *initial_states.get(&(a as i32 + 1, t as i32)).unwrap()
        } else {
            init_idx
        };
        // Set the next task M x A init state [For switch transition] 
        let next_task_idx = if t < nt - 1 {
            *initial_states.get(&(0, t as i32 + 1)).unwrap()
        } else {
            init_idx
        };
        // Add all of the information to the SCPM data structure
        scpm.add_mdp_to_self(&mut mdp, next_agent_idx as usize, next_task_idx as usize);
        // Specify the action space
        let act_start = scpm.actions.start;
        let act_end = scpm.actions.end;
        // Add a placeholder for the threadpool to execute constructing the sparse transition matrix
        // as well as a dense rewards matrix for each action in the action space
        pool.execute(move || {
            SCPM::incremental_construct_spblas_and_rewards(mdp, act_start, act_end, na, nt);
        });
    }
}
// execute and kill the threadpool
pool.join();
```

### Synthesising Schedulers

A scheduler synthesis algorithm is then used to construct the set of schedulers corresponding to points
on the Pareto curve. This forms a downward closure encompassing our target threshold vector $\boldsymbol{t}$.
```rust
// specify the target threshold (costs) for the agents to perform their tasks
let mut target = vec![-65.; na];
// specify the probability threshold for which the agents need to perform the tasks to
let mut ttask = vec![0.99; nt];
target.append(&mut ttask);
// Synthesis the schedulers, and the convex hull of the approximating the multi-objective Pareto 
// curve of this problem
let (mus_, hullset) =
    scpm.imovi_hdd_multi_object_solver(eps, &target[..]);
```

### Visualisation

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [ ] Feature 1
- [ ] Feature 2
- [ ] Feature 3
    - [ ] Nested Feature

See the [open issues](https://github.com/tmrob2/rusty-robots/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the Apache-2.0 License. See `LICENSE` for more information.

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Thomas Robinson - [@twitter_handle](https://twitter.com/twitter_handle) - tmr463@uowmail.edu.au

Project Link: [https://github.com/tmrob2/rusty-robots](https://github.com/tmrob2/rusty-robots)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* []()
* []()
* []()

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/tmrob2/rusty-robots.svg?style=for-the-badge
[contributors-url]: https://github.com/tmrob2/rusty-robots/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/tmrob2/rusty-robots.svg?style=for-the-badge
[forks-url]: https://github.com/tmrob2/rusty-robots/network/members
[stars-shield]: https://img.shields.io/github/stars/tmrob2/rusty-robots.svg?style=for-the-badge
[stars-url]: https://github.com/tmrob2/rusty-robots/stargazers
[issues-shield]: https://img.shields.io/github/issues/tmrob2/rusty-robots.svg?style=for-the-badge
[issues-url]: https://github.com/tmrob2/rusty-robots/issues
[license-shield]: https://img.shields.io/github/license/tmrob2/rusty-robots.svg?style=for-the-badge
[license-url]: https://github.com/tmrob2/rusty-robots/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: img/12x12-4a-9t-collision-avoidance.gif
[mdp-example]: img/product_mdp_message_sending.png
