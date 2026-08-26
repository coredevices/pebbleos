// Start Stride and prove the Watchfaces menu was replaced by a rendered face.
var W = 'http://127.0.0.1:8842';
http.get(W + '/home');
http.get(W + '/tap?text=Watchfaces');
var tap = http.get(W + '/tap?text=Stride');
output.strideInstalled = (tap.status == 200 && json(tap.body).ok) ? 'yes' : 'no';
var seen = '';
var rendered = false;
for (var i = 0; i < 20 && !rendered; i++) {
  seen = json(http.get(W + '/text').body).text.join(' | ');
  rendered = seen != '' && !/Watchfaces/i.test(seen);
}
output.faceText = seen;
output.faceRendered = rendered ? 'yes' : 'no';
http.get(W + '/screenshot?name=02-stride-running');
