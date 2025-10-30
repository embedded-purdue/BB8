
import serial
import json
import threading
import time
import math
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse
import sys
import os
import re
from datetime import datetime
import signal

class graphPID(BaseHTTPRequestHandler):
    
