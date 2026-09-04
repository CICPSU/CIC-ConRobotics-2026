## Part A — ROS Computer Setup

### Step 1 — Log In to the ROS Computer

For the initial setup, you must first log in to the ROS computer physically using your Penn State account.

> **Important:** You only need to complete this initial setup once.

#### 1. Log in to Ubuntu

Log in to the ROS computer using your Penn State account.

Wait until the Ubuntu desktop has fully loaded.

#### 2. Open a Terminal

Click **Show Apps** at the bottom-left corner of the desktop.

<img src="images/step01_show_apps.png" width="900">

Search for **Terminal** and click the **Terminal** application.

<img src="images/step01_terminal_search.png" width="900">
<!-- SCREENSHOT: Application search showing Terminal -->

A Terminal window should open.

<img src="images/step01_terminal_open.png" width="900">
<!-- SCREENSHOT: Open Terminal window -->

You should see a command prompt similar to:

```text
your_psu_id@computer-name:~$
```

Do not close this terminal. You will use this for the next steps.

#### 3. Confirm your user account

First, confirm which user account you are currently using.

Copy and paste the command below into the Terminal. You can use the copy button in the upper-right corner of the code block.

```bash
whoami
```

Press **Enter** and You should see your PSU ID:

```text
your_psu_id
```


### Step 2 — Enable SSH

SSH allows you to remotely access the ROS computer from your own laptop.

First, check whether SSH is enabled:

```bash
systemctl is-enabled ssh
```

If the output is: 
enabled
SSH is already enabled. Continue to the next step.

If the output is:
disabled
enable SSH using:
```bash
sudo systemctl enable ssh
```
Enter your Penn State password if prompted.
Next, start SSH:
```bash
sudo systemctl start ssh
```

Check that SSH is running:
```bash
systemctl is-enabled ssh
```


## Part B — Remote Development Setup

### Step 3 — Create an SSH Key

You will now configure your laptop so that you can remotely access the ROS computer without entering your Penn State password every time.

> **Important:** From this step forward, use **your own laptop**, not the physical ROS computer.

#### 1. Open a Terminal on YOUR Laptop

Open a Terminal **on your laptop**.

- **macOS:** Open **Terminal**.
- **Windows:** Open **PowerShell** or **Windows Terminal**.

#### 2. Check for an Existing SSH Key

Before creating a new SSH key, check whether your laptop already has one.

Copy and paste the following command into the Terminal of your laptop:

```bash
ls ~/.ssh/id_ed25519.pub
```

If you see a file path similar to:

```text
/Users/your_username/.ssh/id_ed25519.pub
```

you already have an SSH key.

> **Do not create a new key.** Continue to **Step 4 — Copy Your SSH Key to the ROS Computer**.

If you see a message similar to:

```text
No such file or directory
```

continue to the next section to create a new SSH key.

#### 3. Generate an SSH Key

Copy and paste the following command into the Terminal:

```bash
ssh-keygen -t ed25519
```

Press **Enter**.

You should see a message similar to:

```text
Generating public/private ed25519 key pair.
Enter file in which to save the key (.../.ssh/id_ed25519):
```

<img src="images/step03_ssh_keygen_location.png" width="900">

Press **Enter** to use the default location.

You will then be asked to enter a passphrase:

```text
Press Enter without typing anything
```

<!-- SCREENSHOT: SSH key passphrase prompt -->

After completing the prompts, you should see a message similar to:

```text
Your identification has been saved in ...
Your public key has been saved in ...
```

<img src="images/step03_ssh_keygen_complete.png" width="900">

#### Checkpoint

Your SSH key has been successfully created.

You are now ready to copy your SSH key to the ROS computer.



### Step 4 — Copy Your SSH Key to the ROS Computer

Next, copy your SSH key to the ROS computer.

This will allow you to connect to the ROS computer without entering your Penn State password every time.

#### 1. Identify the ROS Computer

Make sure you know the IP address of the ROS computer you are using.

```text
10.170.32.181 or 10.170.32.227
```

> **Important:** Use the IP address of the ROS computer assigned to you for the lab day.

#### 2. Copy Your SSH Key

On **your laptop**, run the following command after opening the terminal:

```bash
ssh-copy-id 'YOUR_PSU_ID@AD.PSU.EDU'@ROS_COMPUTER_IP
```

Replace:

- `YOUR_PSU_ID` with your Penn State user ID.
- `ROS_COMPUTER_IP` with the IP address of your assigned ROS computer.

For example:

```bash
ssh-copy-id 'abc123@AD.PSU.EDU'@10.170.32.181
```

Press **Enter**.

#### 3. Confirm the Connection

If this is your first time connecting to the ROS computer, you may see a message similar to:

```text
The authenticity of host '10.170.32.181 (10.170.32.181)' can't be established.
ED25519 key fingerprint is SHA256:...
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

<img src="images/step04_ssh_first_connection.png" width="900">

Type:

```text
yes
```

and press **Enter**.

> **This message is normal.** It appears because your laptop has not connected to this ROS computer before.

#### 4. Enter Your Penn State Password

You may then be asked to enter your Penn State password:

```text
YOUR_PSU_ID@AD.PSU.EDU@ROS_COMPUTER_IP's password:
```

Enter your Penn State password and press **Enter**.

> **Important:** Your password will not appear on the screen while you type. You will not see letters, dots, or asterisks. This is normal.

<img src="images/step04_ssh_password.png" width="900">

If the SSH key was copied successfully, you should see a message similar to:

```text
Number of key(s) added: 1
```

<img src="images/step04_ssh_copy_success.png" width="900">

#### 5. Test the SSH Connection

Now test the connection to the ROS computer.

```bash
ssh 'YOUR_PSU_ID@AD.PSU.EDU'@ROS_COMPUTER_IP
```

For example:

```bash
ssh 'abc123@AD.PSU.EDU'@10.170.32.181
```

Press **Enter**.

If the SSH key was configured correctly, you should connect to the ROS computer without entering your Penn State password.

You should see a Terminal prompt for the ROS computer similar to:

```text
abc123@AD.PSU.EDU@E5-AE-ROS-PC:~$
```

<img src="images/step04_ssh_login_success.png" width="900">

### Checkpoint — Passwordless SSH Connection

Before continuing, confirm that:

- [ ] Your SSH key was successfully copied to the ROS computer.
- [ ] You can connect to the ROS computer from your laptop.
- [ ] You are not asked for your Penn State password when connecting.

> **You are now remotely controlling the ROS computer from your own laptop.**

To return to your laptop, run:

```bash
exit
```

You are now ready to configure **VS Code Remote SSH**.



### Step 5 — Configure VS Code Remote SSH

You will now configure **Visual Studio Code (VS Code)** to remotely access the ROS computer.

After completing this setup, you will be able to use VS Code on your laptop to edit files and run commands directly on the ROS computer.

> **Important:** Complete this step on **your own laptop**.

#### 1. Open VS Code

Open **Visual Studio Code** on your laptop.

<!-- SCREENSHOT: VS Code main window -->

#### 2. Install the Remote - SSH Extension

Click the **Extensions** icon on the left side of VS Code.

<!-- SCREENSHOT: VS Code Extensions icon -->

Search for:

```text
Remote - SSH
```

Install the **Remote - SSH** extension from Microsoft.

<!-- SCREENSHOT: Remote - SSH extension -->

> If the extension is already installed, you do not need to install it again.

#### 3. Open the SSH Configuration File

Open the **Command Palette** in VS Code.

- **macOS:** Press `Command + Shift + P`
- **Windows:** Press `Ctrl + Shift + P`

Search for:

```text
Remote-SSH: Open SSH Configuration File...
```

Select **Remote-SSH: Open SSH Configuration File...**

<!-- SCREENSHOT: Remote-SSH Open SSH Configuration File -->

VS Code may ask you which SSH configuration file you want to open.

Select the file located in your user `.ssh` directory.

It will typically look similar to:

**macOS/Linux**

```text
~/.ssh/config
```

**Windows**

```text
C:\Users\YOUR_USERNAME\.ssh\config
```

<!-- SCREENSHOT: Select SSH configuration file -->

#### 4. Add the ROS Computers

Copy the ROS computer configuration provided below and paste it into the SSH configuration file.

```text

Host dumptruck1
    HostName 10.170.32.192
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck3
    HostName 10.170.32.194
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck4
    HostName 10.170.32.45
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck5
    HostName 10.170.32.219
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator1
    HostName 10.170.32.182
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator2
    HostName 10.170.32.191
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator3
    HostName 10.170.32.222
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host ROS-PC-1
    HostName 10.170.32.181
    User YOUR_PSU_ID@AD.PSU.EDU
    IdentityFile ~/.ssh/id_ed25519

Host ROS-PC-2
    HostName 10.170.32.227
    User YOUR_PSU_ID@AD.PSU.EDU
    IdentityFile ~/.ssh/id_ed25519
```

Replace **YOUR_PSU_ID** with your Penn State user ID.

> **Important:** Do not change `HostName` unless instructed to do so. Only replace `YOUR_PSU_ID` with your own Penn State user ID.

<!-- SCREENSHOT: Completed SSH config -->

Save the file.

- **macOS:** Press `Command + S`
- **Windows:** Press `Ctrl + S`

#### 5. Connect to the ROS Computer

Open the **Command Palette** again:

- **macOS:** `Command + Shift + P`
- **Windows:** `Ctrl + Shift + P`

Search for:

```text
Remote-SSH: Connect to Host...
```

Select **Remote-SSH: Connect to Host...**

<!-- SCREENSHOT: Connect to Host -->

You should now see the ROS computers that you added to the SSH configuration file.

For example:

```text
ROS-PC-1
ROS-PC-2
```

Select the ROS computer assigned to you.

<!-- SCREENSHOT: ROS-PC-1 and ROS-PC-2 host list -->

A new VS Code window should open.

The first connection may take a short time while VS Code configures the remote environment.

If VS Code asks you to select the operating system of the remote computer, select:

```text
Linux
```

<!-- SCREENSHOT: Select Linux -->

#### 6. Confirm the Remote Connection

Look at the **bottom-left corner** of the VS Code window.

You should see an indication that VS Code is connected to your ROS computer.

For example:

```text
SSH: ROS-PC-1
```

<!-- SCREENSHOT: VS Code successfully connected to ROS-PC-1 -->

### Checkpoint — VS Code Remote Connection

Before continuing, confirm that:

- [ ] The **Remote - SSH** extension is installed.
- [ ] Your ROS computers appear in the Remote SSH host list.
- [ ] You can connect to your assigned ROS computer.
- [ ] VS Code shows that you are connected to the ROS computer.

> **Important:** Even though VS Code is running on your laptop, files and commands in this remote VS Code window are now running on the **ROS computer**.

You are now ready to download and build the course repository.


## Part C — Course Repository Setup

### Step 6 — Clone the Course Repository

You will now download the course GitHub repository to the ROS computer.

> **Important:** Before continuing, confirm that VS Code is remotely connected to your assigned ROS computer. Look at the bottom-left corner of VS Code and confirm that you see `SSH: ROS-PC-1` or `SSH: ROS-PC-2`.

#### 1. Open a Terminal in VS Code

From the top menu in VS Code, select:

**Terminal → Split Terminal**

<!-- SCREENSHOT: VS Code Terminal menu with Split Terminal selected -->

A Terminal should open at the bottom of the VS Code window.

<!-- SCREENSHOT: VS Code remote Terminal -->

Look at the Terminal prompt.

It should show the name of the ROS computer, similar to:

```text
your_psu_id@AD.PSU.EDU@E5-AE-ROS-PC:~$
```

> **Important:** This Terminal is running on the **ROS computer**, even though you are using VS Code on your laptop.

#### 2. Go to Your Home Directory

Run:

```bash
cd ~
```

#### 3. Clone the Course Repository

Run:

```bash
git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git
```

Wait for the download to complete.

You should see output similar to:

```text
Cloning into 'CIC-ConRobotics-2026'...
...
```

<!-- SCREENSHOT: Successful git clone -->

#### 4. Enter the Repository

Run:

```bash
cd ~/CIC-ConRobotics-2026
```

Confirm your current location:

```bash
pwd
```

You should see a path ending with:

```text
/CIC-ConRobotics-2026
```

#### 5. Check the Repository

Run:

```bash
ls
```

You should now see the files and folders contained in the course repository.

<!-- SCREENSHOT: Repository contents shown with ls -->

### Checkpoint — Repository Downloaded

Before continuing, confirm that:

- [ ] VS Code is remotely connected to your assigned ROS computer.
- [ ] The course repository was successfully cloned.
- [ ] You are inside the `CIC-ConRobotics-2026` directory.
- [ ] You can see the repository contents using `ls`.

---

### Step 7 — Build the ROS 2 Workspace

Next, you will build the ROS 2 packages used in this course.

#### 1. Set Up ROS 2

Make sure you are inside the course repository:

```bash
cd ~/CIC-ConRobotics-2026
```

Load the ROS 2 Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
```

> The `source` command configures the current Terminal so that it can find and use ROS 2.

#### 2. Build the Workspace

Build the ROS 2 workspace:

```bash
colcon build --symlink-install
```

The build may take some time.

<!-- SCREENSHOT: colcon build running -->

When the build is complete, you should see a summary showing that the ROS 2 packages finished successfully.

<!-- SCREENSHOT: Successful colcon build -->

#### 3. Load the Course Workspace

After the build is complete, run:

```bash
source install/setup.bash
```

> You will need to source the ROS 2 environment and the course workspace when you open a new Terminal. We will automate some of this setup later.

### Checkpoint — ROS 2 Workspace Built

Before continuing, confirm that:

- [ ] `colcon build --symlink-install` completed without errors.
- [ ] The `install` directory was created.
- [ ] You successfully ran `source install/setup.bash`.

You have now downloaded and built the course ROS 2 workspace on the ROS computer.


## Part D — Connect to the Raspberry Pi

### Step 8 — Connect to Your Assigned Raspberry Pi

In this step, you will connect from the ROS computer to a Raspberry Pi used in the course robotics system.

> **Important:** Each student or group must use the Raspberry Pi assigned to them.
>
> Do **not** connect to another group's Raspberry Pi. Multiple students controlling the same Raspberry Pi can interfere with each other's work.

#### 1. Confirm Your Assigned Raspberry Pi

Before connecting, confirm the Raspberry Pi assigned to your group.

Your instructor will provide the Raspberry Pi name and IP address.


<!-- SCREENSHOT OR TABLE: Raspberry Pi assignment list -->

#### 2. Open a Terminal on the ROS Computer

In your VS Code Remote SSH window, open a new Terminal:

**Terminal → New Terminal**

Confirm that you are still connected to the ROS computer.

Your Terminal prompt should look similar to:

```text
your_psu_id@AD.PSU.EDU@E5-AE-ROS-PC:~$
```

#### 3. Open another VSCode window and SSH to the Raspberry Pi

Open anouther VSCode window.

Use SSH to connect to your assigned Raspberry Pi.

Connect through the remote SSH window.

> Your instructor will provide the correct Raspberry Pi username and IP address.

If this is your first time connecting to the Raspberry Pi, you may see:

```text
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

Type:

```text
yes
```

and press **Enter**.

<!-- SCREENSHOT: Raspberry Pi SSH first connection -->

Enter the Raspberry Pi password if prompted.

#### 4. Confirm the Connection

After connecting, your Terminal prompt should change.

For example:

```text
besure@dumptruck1:~$
```

<!-- SCREENSHOT: Successfully connected to Raspberry Pi -->

This means:

```text
Your Laptop
    ↓
VS Code Remote SSH
    ↓
ROS Computer

Your Laptop
    ↓
SSH
    ↓
Raspberry Pi
```

You are now controlling the Raspberry Pi through your computer.

### Checkpoint — Raspberry Pi Connection

Before continuing, confirm that:

- [ ] You are using the Raspberry Pi assigned to your group.
- [ ] You successfully connected to the Raspberry Pi using SSH.
- [ ] Your Terminal prompt now shows the Raspberry Pi hostname.

> **Important:** Keep track of which computer your Terminal is currently controlling. Always check the Terminal prompt if you are unsure.


## Part E — Test ROS 2 Communication

### Step 9 — Test ROS 2 Communication Between Computers

You are now ready to test ROS 2 communication between the ROS computer and Raspberry Pi.

In this test:

- The **ROS computer** will publish messages using a ROS 2 **talker** node.
- The **Raspberry Pi** will receive the messages using a ROS 2 **listener** node.

If the listener receives the messages, ROS 2 communication between the two computers is working.

---

#### 1. Open a Terminal on the ROS Computer

In VS Code, open a **new Terminal**:

**Terminal → New Terminal**

> **Important:** Make sure this Terminal is running on the **ROS computer**, not the Raspberry Pi.

Check the Terminal prompt.

It should look similar to:

```text
your_psu_id@AD.PSU.EDU@E5-AE-ROS-PC:~$
```

Load ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

---

#### 2. Start the ROS 2 Talker

On the **ROS computer**, run:

```bash
ros2 run demo_nodes_cpp talker
```

You should see messages similar to:

```text
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
[INFO] [talker]: Publishing: 'Hello World: 3'
```

<!-- SCREENSHOT: Talker running on ROS computer -->

Keep this Terminal running.

> **Do not close the talker Terminal.**

---

#### 3. Open the other VSCode window

Open another VSCode window and in the Terminal in VS Code:

**Terminal → New Terminal**

This new Terminal should initially be connected to the **RaspberryPi**.

Now connect to your assigned Raspberry Pi:
```

Confirm that the Terminal prompt changes to the Raspberry Pi.

For example:

```text
besure@dumptruck1:~$
```

---

#### 4. Start the ROS 2 Listener

On the **Raspberry Pi**, load ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Then run:

```bash
ros2 run demo_nodes_cpp listener
```

If ROS 2 communication is working correctly, you should begin receiving messages from the talker running on the ROS computer.

You should see output similar to:

```text
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
[INFO] [listener]: I heard: [Hello World: 3]
```

<!-- SCREENSHOT: Listener receiving messages from ROS computer -->

### Checkpoint — ROS 2 Communication

Confirm that:

- [ ] The talker is running on the **ROS computer**.
- [ ] The listener is running on the **Raspberry Pi**.
- [ ] The Raspberry Pi is receiving `Hello World` messages from the ROS computer.

If you can see the messages on the Raspberry Pi:

**Congratulations! You have successfully established ROS 2 communication between two computers.**

---

## Part F — Lab Submission

### Step 10 — Submit Your Result

For this lab, submit **one screenshot** showing successful ROS 2 communication between the ROS computer and Raspberry Pi.

Your screenshot must clearly show:

- The Raspberry Pi Terminal.
- The Raspberry Pi hostname in the Terminal prompt.
- The ROS 2 listener running.
- Multiple `I heard: [Hello World: ...]` messages.

Example:

```text
robot@dumptruck-1:~$ ros2 run demo_nodes_cpp listener

[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
[INFO] [listener]: I heard: [Hello World: 3]
```

<!-- SCREENSHOT: Example of acceptable lab submission -->

### Before You Leave

Stop any running ROS 2 nodes by pressing:

```text
Ctrl + C
```

If you are connected to the Raspberry Pi through SSH, return to your computer using:

```bash
exit
```

You can then close the VS Code Remote SSH connection.

---

# Lab Complete

You have now:

- Configured SSH on the ROS computer.
- Created and installed an SSH key.
- Connected to the ROS computer remotely.
- Configured VS Code Remote SSH.
- Cloned the course GitHub repository.
- Built the ROS 2 workspace.
- Connected to a Raspberry Pi.
- Tested ROS 2 communication between two computers.

You are now ready to use the course robotics system remotely in future labs.



