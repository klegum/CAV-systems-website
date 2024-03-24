from flask import Flask, render_template, jsonify, request
from humanoptimalspeed import main
import os

app = Flask(__name__)

@app.route('/', methods=['GET'])
def index():
    return render_template('cvs.html')


@app.route('/', methods=['POST'])
def sim():
    ah = float(request.form.get("ah"))
    bh = float(request.form.get("bh"))
    main(ah, bh)

    return render_template('cvs.html')
