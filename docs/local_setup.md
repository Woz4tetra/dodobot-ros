# Laptop workstation setup

- Instructions for Ubuntu 20

## Generate SSH keys
- Generate identifier keys: `ssh-keygen`. No passphrase. Default file directory.
```
Generating public/private rsa key pair.
Enter file in which to save the key (~/.ssh/id_rsa): 
Enter passphrase (empty for no passphrase): 
The key fingerprint is:
SHA256:?? ??@??
The key's randomart image is:
??
```

## Setup git

- `apt update`
- `apt install -y git tmux htop`
- Enter identification (replace with account the keys are associated with):
  - `git config --global user.email "your@email.com"`
  - `git config --global user.name "yourusername"`

- `nano ~/.ssh/config`
- Paste the following into the file (replace with private key of your rig):
```
host github.com
  HostName github.com
  IdentityFile ~/.ssh/id_rsa
  User git
```

- Log into https://github.com/
- Go to settings

![alt text](images/GithubMenu.jpg "GithubMenu")
- Navigate to SSH and GPG keys

![alt text](images/GithubSSHkeys.jpg "GithubSSHkeys")

- Click `Add SSH key`
- Paste the contents of the public key file `id_rsa.pub`

![alt text](images/GithubAddKey.jpg "GithubAddKey")

- Click `Add SSH key`

- Clone repository: `git clone git@github.com:Woz4tetra/dodobot-ros.git`
- `cd ./dodobot-ros`
- `git config pull.rebase false`

