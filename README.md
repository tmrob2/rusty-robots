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
  <a href="https://github.com/tmrob2/rusty-robots">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Rusty Robots</h3>

  <p align="center">
    A pure rust implementation for solving multiple-objective task allocation problems in multiagent systems 
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

Here's a blank template to get started: To avoid retyping too much info. Do a search and replace with your text editor for the following: `tmrob2`, `rusty-robots`, `twitter_handle`, `linkedin_username`, `email_client`, `email`, `project_title`, `project_description`

<p align="right">(<a href="#top">back to top</a>)</p>



### Built With

* [Next.js](https://nextjs.org/)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
  ```sh
  npm install npm@latest -g
  ```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/tmrob2/rusty-robots.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_

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
