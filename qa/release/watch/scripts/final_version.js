output.fwFinal = json(http.get('http://127.0.0.1:8842/version').body).tag;
http.get('http://127.0.0.1:8842/screenshot?name=02-final');
