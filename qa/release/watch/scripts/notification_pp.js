// Prove CoreApp's test notification reached the watch by reading the watch screen.
var W = 'http://127.0.0.1:8842';
var seen = '', arrived = false;
for (var i = 0; i < 20 && !arrived; i++) {
  seen = json(http.get(W + '/text').body).text.join(' | ');
  arrived = /test notification|test @/i.test(seen);
}
output.notifText = seen;
output.notifArrived = arrived ? 'yes' : 'no';
http.get(W + '/screenshot?name=02-notification');
http.get(W + '/press?button=back');
