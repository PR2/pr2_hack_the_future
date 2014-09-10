function ConnectionWithOptionalParse(address) {
  ros.Connection.apply(this,arguments);
  this.rawRegExps = new Array();
  var con = this;
  this.socket.oldonmessage = this.socket.onmessage;
  this.socket.onmessage = function(e) {
    for (var idx in con.rawRegExps) {
      var re = con.rawRegExps[idx][0];
      if (!e.data.match(re)) continue;
      var topic = con.rawRegExps[idx][1];
      for (var i in con.handlers[topic]) {
        var handler = con.handlers[topic][i]
        handler(e.data);
      }
      return;
    }
    this.oldonmessage(e);
  }
};
ConnectionWithOptionalParse.prototype = ros.Connection.prototype;
ConnectionWithOptionalParse.prototype.dontParse = function(receiver) {
  this.rawRegExps.push(
    [new RegExp('"receiver":\\s"' + receiver.replace('/','\\/') + '"','gm'),receiver]
  );
}
ros.ConnectionWithOptionalParse = ConnectionWithOptionalParse;
