function LogController(logDivSelector) {

    function addRecord(params, body) {
        let newRecord = $(logDivSelector).find(".template").clone();
        console.log(newRecord);
        newRecord.find(".header").text(params['title']);
        newRecord.removeClass("template");
        let attributeContainer = newRecord.find(".attribute");
        let attributeContainerParent = attributeContainer.parent();
        for (let i = 0; i < body.length; i++) {
            let attribute = body[i];
            console.log(i);
            attributeContainer.find(".name").text(attribute['name']);
            attributeContainer.find(".value").text(attribute['value']);
            attributeContainerParent.append(attributeContainer);
            attributeContainer = attributeContainer.clone();
        }
        $(logDivSelector).prepend(newRecord);

    }

    let that = {
        addRecord: addRecord,
    };

    return that;
}