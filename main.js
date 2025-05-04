const WebSocket = require('ws');
const port = process.env.PORT || 6001;
const wss = new WebSocket.Server({ port }, () =>
  console.log(`Broadcast WS on ${port}`)
);

wss.on('connection', ws => {
  console.log('Client connected');
  ws.on('message', msg => {
    console.log('Recv:', msg);
    // envia para todos
    wss.clients.forEach(c => {
      if (c.readyState === WebSocket.OPEN) c.send(msg);
    });
  });
  ws.on('close', () => console.log('Client disconnected'));
});
