#!/bin/sh

echo convert .ui to .py
pyuic5 -o form.py form.ui
pyuic5 -o form2.py form2.ui


