#!/bin/bash
docker system prune

## Build dockers compose
docker-compose up --build
