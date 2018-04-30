function MainController(logController, mapController) {

    function initStyles() {
        mapController.setStyle('globalRequested', {color: 'blue', weight: 10, opacity: 0.9});
        mapController.setStyle('localRequested', {color: 'blue', weight: 5, opacity: 0.7});
        mapController.setStyle('inWork', {color: 'yellow'});
        mapController.setStyle('confirmed', {color: 'green'});
        mapController.setStyle('rejected', {color: 'red'});
    }

    function styleForServiceType(serviceType) {
        return {"global": "globalRequested", "regional": "localRequested"}[serviceType];
    }

    const valuesToLog = {
        "serviceId": "Service Id:",
        "sender": "Sender:",
        "routeHash": "Route hash:",
        "promisee": "Promisee:",
        "promisor": "Promisor:",
        "liability": "Liability:",
        "isConfirmed": "Is confirmed:"
    };

    function getArgumentsToLog(data) {

        let arguments = [];
        for(let argKey in data) {
            if(data.hasOwnProperty(argKey)) {
                if(argKey in valuesToLog) {
                    arguments.push({"name": valuesToLog[argKey], "value": data[argKey]});
                }
            }
        }
        return arguments;
    }

    function onAsk(data) {
        console.log(data);
        logController.addRecord({'title': 'Ask'}, getArgumentsToLog(data));
        mapController.addRoute(data['routeHash'], data['route'], styleForServiceType(data['serviceType']));
    }

    function onBid(data) {
        logController.addRecord({'title': 'Bid'}, getArgumentsToLog(data));
    }

    function onLiability(data) {
        logController.addRecord({'title': 'Liability created'}, getArgumentsToLog(data));
        mapController.setStyleTo(data['routeHash'], "inWork");
    }

    function onResult(data) {
        logController.addRecord({'title': 'Result gathered'}, getArgumentsToLog(data));
        let style = data['isConfirmed'] ? "confirmed" : "rejected";
        mapController.setStyleTo(data['routeHash'], style);
    }

    function connect() {
        var socket = io.connect('http://' + document.domain + ':' + location.port);
        socket.on('connect', function () {
            console.log("Connected");
            socket.emit('my event', {data: 'I\'m connected!'});
        });
        socket.on('ask', onAsk);
        socket.on('bid', onBid);
        socket.on('liability', onLiability);
        socket.on('result', onResult);
    }

    initStyles();
    mapController.init();
    connect();
}