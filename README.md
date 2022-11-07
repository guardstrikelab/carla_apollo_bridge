<a name="readme-top"></a>


<!-- PROJECT LOGO -->
<br />
<div align="center">
  <!-- <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a> -->

  <h1 align="center">Carla Apollo Bridge</h1>

  <p align="center">
    <b>Apollo & Carla Co-simulation</b>
    <!-- <br /> -->
    <!-- <a href="https://github.com/othneildrew/Best-README-Template"><strong>Explore the docs »</strong></a>
    <br /> -->
    <br />
    <a href="https://github.com/guardstrikelab/apollo_carla">View Demo</a>
    ·
    <a href="https://github.com/guardstrikelab/apollo_carla/issues">Report Bug</a>
    ·
    <a href="https://github.com/guardstrikelab/apollo_carla/pulls">Request Feature</a>
    <br>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<!-- <details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About Carla Apollo Bridge</a>
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
</details> -->


<!-- ABOUT THE PROJECT -->
## About Carla Apollo Bridge
Carla Apollo Bridge aims to provide a bridge for communicating between the latest version of Apollo and Carla.

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

We will run Carla and Apollo in docker. Furthermore, NVIDIA Container Toolkit is needed. You can refer to the following link to install NVIDIA Container Toolkit:
* https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

Alternatively, simply perform the following steps：

* docker

  ```sh
  sudo apt-get install docker.io
  ```
* NVIDIA Container Toolkit

  ```sh
  curl https://get.docker.com | sh \
  && sudo systemctl --now enable docker
  ```
  ```sh
  distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  ```
  ```sh
  sudo apt-get update
  ```
  ```sh
  sudo apt-get install -y nvidia-docker2
  ```
  ```sh
  sudo systemctl restart docker
  ```

### Build And Run Apollo

* Refer to this link：
<br> https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md

1. Clone the apollo repo
   ```sh
   # Using SSH
   git clone git@github.com:guardstrikelab/apollo.git

   #Using HTTPS
   git clone https://github.com/guardstrikelab/apollo.git
   ```
2. Build Apollo

   ```sh
   cd apollo
   git checkout master
   ```
   ```sh
   echo "export APOLLO_ROOT_DIR=$(pwd)" >> ~/.bashrc  && source ~/.bashrc
   ```
   Then, run:
   ```sh
   bash docker/scripts/dev_start.sh
   ```
   to start Apollo development Docker container.
   <br>If you encountered an error, try:
   ```sh
   sudo rm -rf /apollo/.cache
   bash docker/scripts/dev_start.sh
   ```
   After the execution is successful, there will be the following output
   ```sh
   [ OK ] Congratulations! You have successfully finished setting up Apollo Dev Environment.
   [ OK ] To login into the newly created apollo_dev_lei container, please run the following command:
   [ OK ]   bash docker/scripts/dev_into.sh
   [ OK ] Enjoy!
   ```
   Run the command and enter the container:
   ```sh
   bash docker/scripts/dev_into.sh
   ```
   Make the GPU version:
   ```sh
   ./apollo.sh build_gpu
   ```
   After successful compilation, the following will be printed:
   ```sh
   ==============================================
   [ OK ] Done building apollo. Enjoy!
   ==============================================
   ```
   Run the command in the container:
   ```sh
   ./scripts/bootstrap.sh
   ```
   Finnaly, open the link in your browser:
   ```sh
   http://localhost:8888/
   ```

### Run Carla
* Clone apollo_carla_bridge project:
   ```sh
   # Using SSH
   git clone git@github.com:guardstrikelab/apollo_carla_bridge.git

   #Using HTTPS
   git clone https://github.com/guardstrikelab/apollo_carla_bridge.git
   ```

* Install docker-compose

   ```sh
   sudo apt-get install docker-compose
   ```
* Pull carla image and run
  
   ```sh
   cd apollo_carla/carla
   xhost +
   ./docker_run_carla.sh
   ```

<!-- USAGE EXAMPLES -->
## Run Carla_apollo_bridge
1.  Run docker
    ```sh
    cd apollo_carla/carla_apollo_bridge/docker
    ./build_docker.sh
    ./run_docker.sh
    ```
2.  Config
    
    To query the IP address of carla_cyber_0.9.13 container, run the following command outside the container:
    ```sh
    docker inspect carla_cyber_0.9.13 | grep IPAddress
    ```
    Then enter the container:
    ```sh
    docker exec -ti carla_cyber_0.9.13 bash
    vi /apollo/cyber/setup.bash
    ```
    Change the IP address in the bash file to the IP address of the local de container, that is, the IP address obtained above:
    ```sh
    export CYBER_IP=172.17.0.2
    ```
    Update the environment variable in the container:
    ```sh
    source ~/.bashrc
    ```
3.  Compile
    
    Run the following command in the container:
    ```sh
    rm -rf /root/.cache/*
    ./apollo.sh build_cyber opt
    ```
    The following information is displayed after the compilation is successful:
    ```sh
    [INFO] Skipping revision recording
    ============================
    [ OK ] Build passed!
    [INFO] Took 61 seconds
    ```
4. Run

    Run the following command in the container.

    In terminal 1:
    ```sh
    cd /apollo/cyber/carla_bridge
    python carla_cyber_bridge/bridge.py
    ```
    In terminal 2:
    ```sh
    cd /apollo/cyber/carla_bridge
    python carla_spawn_objects/carla_spawn_objects.py
    ```


<!-- ## Usage -->


<!-- ROADMAP -->
<!-- ## Roadmap
- [ ] Add Additional Templates w/ Examples
- [ ] Add "components" document to easily copy & paste sections of the readme
- [ ] Multi-language Support
    - [ ] Chinese
    - [ ] Spanish

See the [open issues](https://github.com/othneildrew/Best-README-Template/issues) for a full list of proposed features (and known issues). -->


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


<!-- LICENSE -->
## License

Distributed under the Apache-2.0 License. See `LICENSE.txt` for more information.

In addition, We have kept the LICENSE of project [Carla Apollo Bridge](https://github.com/AuroAi/carla_apollo_bridge) in carla_apollo_bridge directory.




<!-- CONTACT -->
<!-- ## Contact

Your Name - [@your_twitter](https://twitter.com/your_username) - email@example.com

Project Link: [https://github.com/your_username/repo_name](https://github.com/your_username/repo_name) -->




<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

The co-simulation is modified on the basis of the following, thank you here.

* [Apollo](https://github.com/ApolloAuto/apollo)
* [Carla Apollo Bridge](https://github.com/AuroAi/carla_apollo_bridge)
* [Carla Apollo](https://github.com/casper-auto/carla-apollo)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



