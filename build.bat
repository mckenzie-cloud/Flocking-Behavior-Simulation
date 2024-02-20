@echo off
g++ -O2 -Wall -std=c++2b .\main.cpp -I SFML\include\ -L SFML\lib\ -lsfml-main -lsfml-graphics -lsfml-system -lsfml-window -o bin\app