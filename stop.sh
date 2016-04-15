#! /bin/bash
pid=`ps -A |grep joy_node |awk '{print $1}'`
kill $pid
pid=`ps -A |grep rqt_graph  |awk '{print $1}'`
kill $pid
pid=`ps -A |grep client  |awk '{print $1}'`
kill $pid
