var jsp = null;
var selected_filter = null;
var selected_topic = {"feed1": "None", "feed2": "None"};
var filter = '<div class="filter sort-disabled"><div class="filter-header"></div><div class="filter-body"><div class="filter-inputs"></div><div class="filter-outputs"></div></div></div>';
var filtergroup = '<div class="filtergroup"><div class="filtergroup-header"></div><div class="filtergroup-body"><div class="filter-add left sort-disabled">◁ Add</div>' + filter + '<div class="filter-add right sort-disabled">Add ▷</div></div></div></div>';
var filter_metadata = null;
var filter_picker = $("<select></select>").addClass("filter-picker").change(function() {
    inputs = $(this).closest(".filter").find(".filter-inputs");
    outputs = $(this).closest(".filter").find(".filter-outputs");
    filter_picker_selection = $(this).val();

    jsp.removeAllEndpoints(inputs);
    jsp.removeAllEndpoints(outputs);

    filter_picker_selection = filter_metadata.filter(function(f) {
        return f.name == filter_picker_selection;
    });

    for(i in filter_picker_selection[0].inputs) {
        jsp.addEndpoint(inputs,
            { anchor:[0.5, (parseInt(i) + 1) * (1.0 / (filter_picker_selection[0].inputs.length + 1)), -1, 0] },
            {
                endpoint: ["Dot", {radius: 5} ],
                cssClass: "input",
                hoverClass: "input-hover",
                paintStyle: { fillStyle: "#FF0000", opacity: 0.5 },
                isTarget: true,
                scope: filter_picker_selection[0].inputs[i].type,
                overlays:[
                    [ "Label", { location: [0, -1.2], label: filter_picker_selection[0].inputs[i].type, cssClass: "input-label" } ]
                ]
            }
        );
    }

    for(i in filter_picker_selection[0].outputs) {
        jsp.addEndpoint(outputs,
            { anchor:[0.5, (parseInt(i) + 1) * (1.0 / (filter_picker_selection[0].outputs.length + 1)), 1, 0] },
            {
                endpoint: ["Dot", {radius: 5} ],
                cssClass: "output",
                hoverClass: "output-hover",
                paintStyle: { fillStyle: "#0000FF", opacity: 0.5 },
                isSource: true,
                scope: filter_picker_selection[0].outputs[i].type,
                maxConnections: -1,
                connector: [ "Bezier" ],
                connectorStyle: {
                    gradient: { stops: [[0, "#00F"], [0.5, '#09098e'], [1, "#00F"]] },
                    strokeStyle: "#00F",
                    lineWidth: 5
                },
                overlays:[
                    [ "Label", { location: [0, -1.2], label: filter_picker_selection[0].outputs[i].type, cssClass: "output-label" } ]
                ]
            }
        );
    }
});
var topic_input = function (text) { return '<a href="#" class="list-group-item list-group-item-success">' + text + '</a>'; };

function init_filtergroup(new_filtergroup) {
        $(new_filtergroup).find(".filtergroup-body").sortable({
        tolerance: "pointer",
        scroll: false,
        items: ".filter",
        cancel: ".sort-disabled,.filter-picker",
        connectWith: ".filtergroup-body",
        containment: "#workspace",
        placeholder: "filter-placeholder",
        sort: function(event, ui) {
            update_sortable_filters();
        },
        stop: function(event, ui) {
            update_sortable_filters();
        },
        update: function(event, ui) {
            update_sortable_filters();
        }
    });

    init_filter(new_filtergroup.find(".filter"));
}

function init_filter(new_filter) {
    update_sortable_filters();

    new_filter.find(".filter-header").append($(filter_picker).clone(true));

    new_filter.click(function() {
        if(selected_filter != null) {
            selected_filter.removeClass("bg-info");
        }

        $(this).addClass("bg-info");
        selected_filter = $(this);

    });
}

function update_sortable_filters() {
    $.each($(".filtergroup-body"), function(index, filtergroup) {
        if($(filtergroup).find(".filter").length == 1) {
            $(filtergroup).find(".filter").addClass("sort-disabled");
        } else {
            $(filtergroup).find(".filter").removeClass("sort-disabled");
        }
    });

    jsp.repaintEverything();
}

function draw_image(canvas_id, imgString){
    var canvas = $("#" + canvas_id)[0];
    var ctx = canvas.getContext("2d");
    var image = new Image();

    if(imgString != "None") {
        image.src = ((imgString != null) ? "data:image/jpg;base64," + imgString : null);

    } else {
        image.src = "images/no_feed.png";
    }

    image.onload = function() {
        // FIXME: Auto adjust canvas dimensions
        /*canvas.style.width='100%';
        canvas.style.height='100%';
        canvas.width  = canvas.offsetWidth;
        canvas.height = canvas.offsetHeight;*/
        ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
    };
}

function update_topic_list(topic_selector_id, data) {
    $("#" + topic_selector_id + "-topics").empty();
    $("#" + topic_selector_id + "-topics").append('<option value="None">None</option>');

    topics = JSON.parse(data);

    $.each(topics, function(i, optgroups) {
        $.each(optgroups, function(groupName, options) {
            var $optgroup = $("<optgroup>", {label: groupName});
            $optgroup.appendTo($("#" + topic_selector_id + "-topics"));

            $.each(options, function(j, option) {
                var $link = $("<a>", {text: option, class: "list-group-item"});
                var $option = $("<option>", {text: option, value: groupName});

                $option.appendTo($optgroup);
                if($("#" + groupName.split("/").pop().toLowerCase() + "-inputs" + " > a:contains(" + option + ")").length == 0) {
                    jsp.makeSource($(topic_input(option)).appendTo("#" + groupName.split("/").pop().toLowerCase() + "-inputs"),
                        {
                            anchor: "Right",
                            scope: groupName,
                            connector: [ "Flowchart", { stub: [10, 10], alwaysRespectStubs: true, midpoint: 0.05 } ],
                            connectorStyle: {
                                strokeStyle: "#0000FF",
                                lineWidth: 4
                            }
                        });
                }
            });
        });
    });

    $("#" + topic_selector_id + "-topics option:contains(" + selected_topic[topic_selector_id] + ")").attr('selected', true);
}

jsPlumb.ready(function() {
    var input1 = new WebSocket("ws://localhost:8888/input1/");
    var input2 = new WebSocket("ws://localhost:8888/input2/");
    var topics = new WebSocket("ws://localhost:8888/topics/");
    var filters = new WebSocket("ws://localhost:8888/filters/");
    var load = new WebSocket("ws://localhost:8888/load/");
    //var save = new WebSocket("ws://localhost:8888/save/");


    draw_image("feed1", "None");
    draw_image("feed2", "None");

    $(window).resize(function() {
        jsp.repaintEverything();
    });

    input1.onmessage = function(evt) {
        draw_image("feed1", evt.data);
    };

    input2.onmessage = function(evt) {
        draw_image("feed2", evt.data);
    };

    topics.onmessage = function(evt) {
        update_topic_list("feed1", evt.data);
        update_topic_list("feed2", evt.data);
    };

    filters.onmessage = function(evt) {
        filter_metadata = JSON.parse(evt.data)
        $(filter_picker).append($("<option selected disabled>Pick a filter type...</option>"));

        $.each(filter_metadata, function(i, f) {
             $(filter_picker).append($("<option></option>").val(f.name).text(f.name));
        });
    };

    load.onmessage = function(evt) {
        $("#load-filterchain").prop('disabled', true);
        $("#filterchain-picker").empty();
        $("#filterchain-picker").append($("<option selected disabled>Pick a filter chain...</option>"));

        $("#filterchain-picker").change(function() {
            $("#load-filterchain").prop('disabled', false);
        });

        $.each(JSON.parse(evt.data), function(i, fc) {
             $("#filterchain-picker").append($("<option></option>").val(fc).text(fc));
        });
    };

    $("#load-filterchain").click(function () {
        $( "#load-dialog" ).dialog({
            resizable: false,
            height: 220,
            modal: true,
            buttons: {
                "Load Filter Chain": function() {
                    $( this ).dialog( "close" );
                    load.send($("#filterchain-picker").val());
                },
                Cancel: function() {
                    $( this ).dialog( "close" );
                }
            }
        });
    });

    $("#feed1-topics").change(function () {
        selected_topic["feed1"] = $("option:selected", this).text();
        input1.send([$("option:selected", this).text(), $("option:selected", this).val()]);
    });

    $("#feed2-topics").change(function () {
        selected_topic["feed2"] = $("option:selected", this).text();
        input2.send([$("option:selected", this).text(), $("option:selected", this).val()]);
    });

    $(".filtergroup-add.top").click(function() {
        var new_filtergroup = $(filtergroup.toString());

        $(this).after(new_filtergroup);
        init_filtergroup(new_filtergroup);
    });

    $(".filtergroup-add.bottom").click(function() {
        var new_filtergroup = $(filtergroup.toString());

        $(this).before(new_filtergroup);
        init_filtergroup(new_filtergroup);
    });

    $("body").on("click", ".filter-add.right", function() {
        var new_filter = $(filter.toString());

        $(this).before(new_filter);
        init_filter(new_filter);
    });

    $("body").on("click", ".filter-add.left", function() {
        var new_filter = $(filter.toString());

        $(this).after(new_filter);
        init_filter(new_filter);
    });

	jsp = jsPlumb.getInstance({
		Endpoint : ["Dot", {radius:2}],
		HoverPaintStyle : {strokeStyle:"#1e8151", lineWidth:2 },
		ConnectionOverlays : [
			[ "Arrow", {
				location:1,
				id:"arrow",
                length:14,
                foldback:0.8
			} ],
            [ "Label", { label:"FOO", id:"label", cssClass:"aLabel" }]
		],
		Container:"workspace"
	});

    jsp.doWhileSuspended(function() {
        jsp.bind("connection", function(info) {
            info.connection.getOverlay("label").setLabel(info.connection.id);
        });
    });
});