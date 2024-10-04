#!/bin/bash
docker system prune -f

## Build dockers compose
docker-compose up --build
