#!/bin/bash
pgrep $1 2>&1 > /dev/null

if [ $? -eq 0 ]
then
{
echo "process running"
ps -ef | grep $1 | grep -v grep | awk '{print $2}'| xargs kill -9
}
else
{
echo "no process running"
};fi

