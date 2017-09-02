from flask import Flask
from flask import jsonify
from flask import request


app = Flask(__name__)


@app.route('/')
def index():
    print(request.data)
    return "Hello, World!"


@app.route('/send')
def send():
    ret = {
        "action": "send"
    }
    data = request.data
    print(data)
    if data is None:
        return jsonify(ret), 400
    return jsonify(ret), 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
