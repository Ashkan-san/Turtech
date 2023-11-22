import os

from paramiko import SSHClient
from scp import SCPClient


def copy_to_vm(directory="search/see/projectlp"):
    ssh = SSHClient()
    ssh.load_system_host_keys()
    ssh.connect('192.168.167.54', username="projectuser", password="J34v2mdX")

    # SCPCLient takes a paramiko transport as an argument
    scp = SCPClient(ssh.get_transport())

    scp.put(directory, recursive=True, remote_path='/home/projectuser/')

    # Uploading the 'test' directory with its content in the
    # '/home/user/dump' remote directory
    # scp.put('test', recursive=True, remote_path='/home/user/dump')

    scp.close()
    clean_local_projectlp(directory)


def clean_local_projectlp(directory):
    clean_directory(f"{directory}/location")
    clean_directory(f"{directory}/photos")


def clean_directory(path: str):
    files = os.listdir(path)
    for file in files:
        os.remove(path + "/" + file)
        print("deleted: " + path + "/" + file)


if __name__ == "__main__":
    clean_local_projectlp()
