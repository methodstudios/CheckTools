name = "CheckTools"

version = "1.1.0"

authors = [
    "github.com/minoue",
    "smackenzie",
    "barejkop",
]

description = \
    """
    Mesh/UV check commands for Maya
    """

tools = [
]

requires = [
]

private_build_requires = [
    'cmake-3.12',
    'git-2',
    'gcc-4',
]

variants = [
    ["platform-linux", "arch-x86_64", "os-CentOS-7", "maya-2016.5"],
    ["platform-linux", "arch-x86_64", "os-CentOS-7", "maya-2017.0"],
    ["platform-linux", "arch-x86_64", "os-CentOS-7", "maya-2017.4"],
    ["platform-linux", "arch-x86_64", "os-CentOS-7", "maya-2017.5"],
    ["platform-linux", "arch-x86_64", "os-CentOS-7", "maya-2018"],
    ["platform-linux", "arch-x86_64", "os-CentOS-7", "maya-2019"],
]

uuid = '36fe4f69-23eb-4467-81ec-a5e15922109b'

with scope('config') as config:
    config.release_packages_path = '${METHOD_REZ_EXTERNAL_PACKAGES}'

def commands():
    env.MAYA_PLUG_IN_PATH.append('{this.root}/plug-ins')

