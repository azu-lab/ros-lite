#!/bin/sh

# Please source this file to use ROS-lite Command Line Interface

CURRENT_DIR=$(cd $(dirname $BASH_SOURCE); pwd)
export PATH="$CURRENT_DIR/roslite_cli:$CURRENT_DIR/roslite_cli/generated:$PATH"

chmod +x $CURRENT_DIR/roslite_cli/*
chmod +x $CURRENT_DIR/roslite_cli/generated/*
chmod +x $CURRENT_DIR/*

_hoge_test()
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    # COMPREPLY=( standard beta dev canary )
    case "$COMP_CWORD" in
    1)
        COMPREPLY=( $(compgen -W "$(ls $CURRENT_DIR/../../ros_src/msg)" -- $cur) );;
    *)
        COMPREPLY=( $(compgen -W "standard beta dev canary" -- $cur) );;
    esac
}
# complete -F _hoge_test hoge

_rosl_build()
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    case "$COMP_CWORD" in
    *)
        COMPREPLY=( $(compgen -W "$(ls $CURRENT_DIR/../../ros_src/map) clean depend" -- $cur) );;
    esac
}
complete -F _rosl_build rosl_build

_rosl_map_gen()
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=( $(compgen -W "$(ls $CURRENT_DIR/../../ros_src/map)" -- $cur) )
}
complete -F _rosl_map_gen rosl_map_gen

_rosl_msg_gen()
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=( $(compgen -W "$(find $CURRENT_DIR/../../ros_src/msg -name "*.msg" | sed "s#$CURRENT_DIR/../../ros_src/msg/##g")" -- $cur) )
}
complete -F _rosl_msg_gen rosl_msg_gen

alias rosl_cd='cd $CURRENT_DIR/../../../../'
