# API

A Python API for controlling the robot platform
```
API/
├── cfscl/             ## CFServo SDK usage example
├── cfservo_sdk/       ## CFServo SDK for Waveshare servos
├── apto_sdk.py        ## Robot platform API
├── main.py            ## Apto SDK usage
└── requirements.txt
```

## Setup

(Optional) Create virtual environment

```bash
conda create -n apto_env python==3.12
```

Install dependencies

```bash
pip install -r requirements.txt
```

## Usage

Run example script to move robot to home pose

```bash
python main.py
```