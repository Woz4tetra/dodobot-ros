# Generate ssh key-pair

Set ROBOT_NAME

`export ROBOT_NAME=...`

Echo key location (for `Enter file in which to save the key`)

`echo /home/${USER}/.ssh/${ROBOT_NAME}`

Generate an SSH key

`ssh-keygen`

```(bash)
Generating public/private rsa key pair.
Enter file in which to save the key (/home/${USER}/.ssh/id_rsa): /home/${USER}/.ssh/${ROBOT_NAME}
Created directory '/home/${USER}/.ssh'.
Enter passphrase (empty for no passphrase):
Enter same passphrase again:
Your identification has been saved in /home/${USER}/.ssh/${ROBOT_NAME}.
Your public key has been saved in /home/${USER}/.ssh/${ROBOT_NAME}.pub.
The key fingerprint is:
SHA256:-----------------------/------------------- ${USER}@${ROBOT_NAME}
The key's randomart image is:
```

# Setup SSH key directories
Copy `/home/${USER}/.ssh/${ROBOT_NAME}.pub` to `/home/${USER}/.ssh/authorized_keys`

`cp /home/${USER}/.ssh/${ROBOT_NAME}.pub /home/${USER}/.ssh/authorized_keys`


Copy `${ROBOT_NAME}` and `${ROBOT_NAME}.pub` to your local machine's `~/.ssh` directory.

Change permissions of these files to 600: `chmod 600 ${ROBOT_NAME}*`

Test the log in: `ssh -i ~/.ssh/${ROBOT_NAME} ${USER}@${ROBOT_NAME}.local`

# Disable password login
`sudo nano /etc/ssh/sshd_config`

Search for `#PasswordAuthentication yes`

Change to `PasswordAuthentication no`

Save and restart the ssh service:

`sudo service ssh restart`
