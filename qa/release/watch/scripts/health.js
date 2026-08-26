// Proves the Health app renders. Step detection itself requires human motion or a shaker rig.
var W = 'http://127.0.0.1:8842';
http.get(W + '/home');
http.get(W + '/tap?text=Health');
output.healthText = json(http.get(W + '/text').body).text.join(' | ');
http.get(W + '/screenshot?name=02-health');
http.get(W + '/press?button=back');
