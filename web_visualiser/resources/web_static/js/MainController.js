function MainController(logController, mapController) {

    function initStyles() {
        mapController.setStyle('globalRequested', {color: 'blue', weight: 10, opacity: 0.9});
        mapController.setStyle('localRequested', {color: 'blue', weight: 5, opacity: 0.7});
        mapController.setStyle('inWorkGlobal', {color: 'yellow'});
        mapController.setStyle('inWorkLocal', {color: 'deepskyblue'});
        mapController.setStyle('confirmed', {color: 'green'});
        mapController.setStyle('rejected', {color: 'red'});
    }

    function styleForServiceType(serviceType, styleType) {
        return {
            "requested": {
                "global": "globalRequested", "regional": "localRequested"
            },
            "inWork": {
                "global": "inWorkGlobal", "regional": "inWorkLocal"
            }
        }[styleType][serviceType];
    }

    const valuesToLog = {
        "serviceId": "Service Id:",
        "model": "Service model:",
        "liability": "Liability:",
        "sender": "Sender:",
        "routeHash": "Route hash:",
        "promisee": "Promisee:",
        "promisor": "Promisor:",
        "isConfirmed": "Is confirmed:"
    };

    function getArgumentsToLog(data) {
        let arguments = [];
        for (let argKey in valuesToLog) {
            if (valuesToLog.hasOwnProperty(argKey)) {
                if (argKey in data) {
                    arguments.push({"name": valuesToLog[argKey], "value": data[argKey]});
                }
            }
        }
        return arguments;
    }

    function onAsk(data) {
        logController.addRecord({'title': 'Ask', 'bgColor': 'blue'}, getArgumentsToLog(data));
        mapController.addRoute(data['routeHash'], data['route'], styleForServiceType(data['serviceType'], 'requested'));
    }

    function onBid(data) {
        logController.addRecord({'title': 'Bid', 'bgColor': 'deepskyblue'}, getArgumentsToLog(data));
    }

    function onLiability(data) {
        logController.addRecord({'title': 'Liability created', 'bgColor': 'orange'}, getArgumentsToLog(data));
        mapController.setStyleTo(data['routeHash'], styleForServiceType(data['serviceType'], 'inWork'));
    }

    function onResult(data) {

        let style = data['isConfirmed'] ? "confirmed" : "rejected";
        let color = data['isConfirmed'] ? "green" : "red";
        logController.addRecord({'title': 'Result gathered', 'bgColor': color}, getArgumentsToLog(data));
        mapController.setStyleTo(data['routeHash'], style);
    }

    function connect() {
        let socket = io.connect('http://' + document.domain + ':' + location.port);
        socket.on('ask', onAsk);
        socket.on('bid', onBid);
        socket.on('liability', onLiability);
        socket.on('result', onResult);
    }

    initStyles();
    mapController.init();
    connect();
}