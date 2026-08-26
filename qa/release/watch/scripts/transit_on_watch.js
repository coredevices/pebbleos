// Start Transit and prove it renders a schedule for the hardware lab's deterministic location.
var W = 'http://127.0.0.1:8842';
function launch() { http.get(W + '/home'); http.get(W + '/tap?text=Transit'); }
launch();
var seen = '', ok = false, relaunches = 0;
for (var i = 0; i < 30 && !ok; i++) {
  seen = json(http.get(W + '/text').body).text.join(' | ');
  if (/failed/i.test(seen) && relaunches < 2) { relaunches++; launch(); continue; }
  ok = /Caltrain|Palo Alto/i.test(seen);
}
output.transitText = seen;
output.transitCaltrain = ok ? 'yes' : 'no';
http.get(W + '/screenshot?name=02-transit');
http.get(W + '/press?button=back');
