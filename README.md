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
sequential composition product MDP,
which efficiently computes the task allocation and a set of schedulers which agents can execute 
concurrently. 

The following gif demonstrates the allocation of tasks to a set of agents in a warehouse simulation as
a high level demonstration of the MOTAP capabilities. In this diagram, 30 tasks are allocated to 20 agents 
while satisfying the multiobjective tradeoff between the cost required for an agent to execute a set of tasks,
and the probability requirement of completing of tasks. 


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

### Prerequisites

To solve a MOTAP problem we require `motap-hdd` from https://github.com/tmrob2/motap-hdd. Follow the
setup instructions of this project. This will require seting up `Gurobi Optimizer` and `CXSparse` as a
sparse matrix solver. 

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

### Constructing a simulation model
This project is designed as a helper API for configuring large scale task allocation problems. First we
require some environment. An environment can be constructed as follows. Environments can be added to the
`env` directory. 

### Constructing a task
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


### Constructing a rewards model

### Forming a product MDP

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
[license-url]: https://github.com/tmrob2/rusty-robots/blob/master/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: img/warehouse_example.gif
