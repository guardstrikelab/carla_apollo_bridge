import subprocess
import sys
import os
import time
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description='Post your docker name.')

    parser.add_argument('-n', type=str, required=True, help='Input for apollo_docker_name')
    parser.add_argument('-f', type=str, required=True, help='Input for record_file')

    args = parser.parse_args()
    
    return args.n, args.f


def main():
    # send routing_request to apollo
    docker_container_name, record_file_path = parse_arguments()
    docker_cyber_recorder_command = "docker exec -it {} bash -c 'source cyber/setup.bash; cyber_recorder play -f {}'".format(docker_container_name, record_file_path)
    process = subprocess.Popen(docker_cyber_recorder_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    for line in process.stdout:
        print(line.decode(), end='')
    for line in process.stderr:
        print(line.decode(), end='')
    time.sleep(5)

if __name__ == "__main__":
    main()
