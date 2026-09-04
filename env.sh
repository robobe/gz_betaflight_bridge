#!/usr/bin/env bash

alias gzps="pgrep -af '^gz sim( |$)'"
alias gzkill="pkill -9 -f '^gz sim( |$)'"

source /usr/lib/git-core/git-sh-prompt
PS1='\[\e[32m\]\u@\h\[\e[0m\]:\[\e[34m\]\w\[\e[33m\]$(__git_ps1 " (%s)")\[\e[0m\]\$ '
