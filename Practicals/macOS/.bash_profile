# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/usr/local/Caskroom/mambaforge/base/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/usr/local/Caskroom/mambaforge/base/etc/profile.d/conda.sh" ]; then
        . "/usr/local/Caskroom/mambaforge/base/etc/profile.d/conda.sh"
    else
        export PATH="/usr/local/Caskroom/mambaforge/base/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<

# PERSONAL MODIFICATIONS
# --------------------------------------------------------
alias ll="ls -l --color"
alias lla="ls -al --color"

# bash-completion (has to be installed with brew)
[[ -r "/usr/local/opt/bash-completion/etc/profile.d/bash_completion.sh" ]] && . "/usr/local/opt/bash-completion/etc/profile.d/bash_completion.sh"

# prompt update
source $HOME/.bash_prompt

# Search in history
bind '"\e[A": history-search-backward'
bind '"\e[B": history-search-forward'
HISTSIZE=20000
HISTFILESIZE=20000

# ROS
# --------------------------------------------------------
conda activate robot_3.9 # Change to the environment name
export TURTLEBOT3_MODEL="burger"
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

source /usr/local/Caskroom/mambaforge/base/envs/robot_3.9/setup.bash
source /Users/sylar/catkin_ws/devel/setup.bash # Change to correct path
