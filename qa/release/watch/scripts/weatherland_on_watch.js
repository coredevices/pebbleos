// Start Weather Land and prove that its date and temperature slot render after PebbleKit-JS data.
var W = 'http://127.0.0.1:8842';
function launch() {
  http.get(W + '/home');
  http.get(W + '/tap?text=Watchfaces');
  http.get(W + '/tap?text=Weather');
}
function weatherRendered(arr) {
  var joined = arr.join(' | ');
  var hasDate = /\b(AUG|JAN|FEB|MAR|APR|MAY|JUN|JUL|SEP|OCT|NOV|DEC)\b/i.test(joined);
  var hasTemp = arr.some(function (t) {
    var s = t.trim();
    return s.length >= 1 && s.length <= 3 && s.indexOf(':') < 0 && /[0-9]/.test(s) &&
           !/(AUG|JAN|FEB|MAR|APR|MAY|JUN|JUL|SEP|OCT|NOV|DEC|THU|MON|TUE|WED|FRI|SAT|SUN)/i.test(s);
  });
  return hasDate && hasTemp;
}
launch();
var seen = '', ok = false, relaunches = 0;
for (var it = 0; it < 40 && !ok; it++) {
  var reader = (it % 2 === 0) ? '' : '?reader=llm';
  var body = json(http.get(W + '/text' + reader).body);
  seen = body.text.join(' | ');
  if (/failed/i.test(seen) && relaunches < 2) { relaunches++; launch(); continue; }
  ok = weatherRendered(body.text);
}
output.weatherText = seen;
output.weatherTemp = ok ? 'yes' : 'no';
http.get(W + '/screenshot?name=02-weather-land');
