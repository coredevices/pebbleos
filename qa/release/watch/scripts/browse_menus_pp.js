// Release firmware has no readable window stack, so prove navigation by reading each screen.
var W = 'http://127.0.0.1:8842';
function open(name) {
  http.get(W + '/tap?text=' + name);
  return json(http.get(W + '/text').body).text.join(' | ');
}
http.get(W + '/home');
open('Settings');
output.mBluetooth = open('Bluetooth');
http.get(W + '/screenshot?name=02-menu-bluetooth');
http.get(W + '/press?button=back');
output.mNotifications = open('Notifications');
http.get(W + '/screenshot?name=02-menu-notifications');
http.get(W + '/press?button=back');
output.mSystem = open('System');
http.get(W + '/screenshot?name=02-menu-system');
http.get(W + '/press?button=back');
