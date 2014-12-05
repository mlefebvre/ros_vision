var jsp = null;
var external_input_sources = []
var selected_filter = null;
var selected_topic = {"feed1": "None", "feed2": "None"};
var filter = '<div class="filter sort-disabled panel panel-default"><div class="filter-header panel-heading"><span class="filter-type"></span><span class="filter-name"></span><span class="filter-controls btn-group"><button class="edit-filter btn btn-default" type="button"><span class="glyphicon glyphicon-pencil"></span></button><button class="delete-filter btn btn-default" type="button"><span class="glyphicon glyphicon-remove"></span></button></span></div><div class="filter-body panel-body"><div class="filter-inputs"></div><div class="filter-outputs"></div></div></div>';
var filter_picker = $('<ul class="dropdown-menu" role="menu"></ul>').on('click', 'li a', function() {
        var new_filter = $(filter.toString());

        if($(this).closest(".add-filter-container").hasClass("left")) {
            $(this).closest(".filtergroup-body").children(".inactive:nth-child(2)").remove();
            $(this).closest(".add-filter-container").after(new_filter);
        } else {
            $(this).closest(".filtergroup-body").children(".inactive:nth-last-child(2)").remove();
            $(this).closest(".add-filter-container").before(new_filter);
        }

        init_filter(new_filter);
        pad_filter_groups();
});
var filtergroup = '<div class="filtergroup"><div class="filtergroup-header input-group input-group-lg"><span class="filtergroup-name input-group-addon"></span><span class="input-group-btn"><button class="edit-filtergroup btn btn-default" type="button"><span class="glyphicon glyphicon-pencil"></span></button><button class="delete-filtergroup btn btn-default" type="button"><span class="glyphicon glyphicon-remove"></span></button></span></div><div class="filtergroup-body well well-sm"><div class="add-filter-container left sort-disabled btn-group"><button type="button" class="add-filter left btn btn-default dropdown-toggle" data-toggle="dropdown" aria-expanded="false"><span class="glyphicon glyphicon-plus"></span></button></div><div class="add-filter-container right sort-disabled"><button type="button" class="add-filter right btn btn-default dropdown-toggle" data-toggle="dropdown" aria-expanded="false"><span class="glyphicon glyphicon-plus"></span></button></div></div></div>';
var filter_metadata = null;
var topic_input = function (text) { return '<a href="#" class="list-group-item list-group-item-success">' + text + '</a>'; };
var parameter = function (name) { return '<div class="form-group"><label for="' + name + '" class="parameter-name control-label">' + name + '</label><div class="input-group"><span class="input-group-btn"><button class="reset-parameter btn btn-default" type="button">Reset</button></span></div></div>'; }

function load_filter_chain(workspace) {
    // Clean up client
    jsp.reset();
    $(".filtergroup").remove();

    // Initialize external inputs
    for(input_topic_index in workspace.input_topics) {
        var topic_name = workspace.input_topics[input_topic_index].topic;
        var topic_type = workspace.input_topics[input_topic_index].type;

        if($("#" + topic_type.split("/").pop().toLowerCase() + "-inputs" + " > a:contains(" + topic_name + ")").length == 0) {
            var input = $(topic_input(topic_name)).appendTo("#" + topic_type.split("/").pop().toLowerCase() + "-inputs");

            input.attr("id", topic_name.replace(/\//g, '_'));
            external_input_sources.push(jsp.makeSource(input,
                {
                    anchor: "Right",
                    scope: topic_type,
                    connector: [ "Flowchart", { stub: [10, 10], alwaysRespectStubs: true, midpoint: 0.05 } ],
                    connectorStyle: {
                        strokeStyle: "#0000FF",
                        lineWidth: 4
                    }
                }));
        }
    }

    // Initialize filters and filter groups
    for(group_index in workspace.filter_groups) {
        var new_filtergroup = $(filtergroup.toString());

        $(".filtergroup-add.bottom").before(new_filtergroup);
        init_filtergroup(new_filtergroup, {name: workspace.filter_groups[group_index].name});

        for(filter_index in workspace.filter_groups[group_index].filters) {
            var new_filter = $(filter.toString());

            new_filtergroup.find(".add-filter-container.right").before(new_filter);
            init_filter(new_filter, workspace.filter_groups[group_index].filters[filter_index]);
        }
    }

    // Initialize all connections
    for(group_index in workspace.filter_groups) {
        for(filter_index in workspace.filter_groups[group_index].filters) {
            init_connections(workspace.filter_groups[group_index].filters[filter_index].inputs, workspace.filter_groups[group_index].name, workspace.filter_groups[group_index].filters[filter_index].name);
        }
    }

    optimize_filter_positions();
    pad_filter_groups();
}

function init_filter_properties(type, description, parameters) {
    $("#filter-type-name").text(type);
    $("#filter-type-description").text(description);
    $("#filter-properties").empty();

    $.each(parameters, function (i, p) {
        var parameter_container = $(parameter(p.name));

        switch(p.type) {
            case "str":
                // FIXME: This shouldn't be in the "min" parameter...
                var select_options = null;

                try {
                    select_options = JSON.parse(p.min.replace(/'/g, "\""));
                } catch(err) { }

                if($.isArray(select_options)) {
                    parameter_container.find(".input-group").prepend($('<select id="' + p.name + '" class="parameter-value form-control" />'));

                    $.each(select_options, function (index, option) {
                        parameter_container.find("#" + p.name).append($("<option>", {text: option, value: option}));
                    });

                    parameter_container.find("#" + p.name).val(p.default);

                    parameter_container.find("#" + p.name).change(function () {
                        // TODO: Send to master
                    });
                } else {
                    parameter_container.find(".input-group").prepend($('<input id="' + p.name + '" type="text" class="parameter-value form-control" value="' + p.default + '" />'));

                    parameter_container.find("#" + p.name).on("input", function () {
                        // TODO: Send to Master
                    });
                }

                parameter_container.find(".reset-parameter").click(function() {
                    $("#" + p.name).val(p.default);
                });
                break;

            case "float":
            case "int":
                parameter_container.find(".input-group").prepend($('<input id="' + p.name + '" type="text" class="parameter-value form-control" value="' + p.default + '" />'));

                if($.isNumeric(p.min) && $.isNumeric(p.max)) {
                    parameter_container.find(".input-group").before($('<div id="' + p.name + '-slider" class="slider"></div>'));
                }

                parameter_container.find("#" + p.name).on("input", function () {
                    // TODO: Send to Master
                });

                parameter_container.find(".reset-parameter").click(function() {
                    $("#" + p.name).val(p.default);
                });
                break;

            case "bool":
                parameter_container.find(".input-group").prepend($('<input id="' + p.name + '" type="checkbox" class="parameter-value form-control" />'));
                $("#" + p.name).prop('checked', p.default);

                parameter_container.find(".reset-parameter").click(function() {
                    $("#" + p.name).prop('checked', p.default);
                });

                parameter_container.find("#" + p.name).change(function () {
                    // TODO: Send to Master
                });
                break;
        }

        $("#filter-properties").append(parameter_container);

        if($.isNumeric(p.min) && $.isNumeric(p.max)) {
            var step = (p.type == "float") ? 0.1 : 1;

            $("#" + p.name + "-slider").slider({
                value: p.default,
                min: parseInt(p.min),
                max: parseInt(p.max),
                step: step,
                slide: function( event, ui ) {
                    $("#" + p.name).val(ui.value);
                }
            });
        }
    });
}

function optimize_filter_positions() {
    $(".filter.inactive").remove();

    jsp.selectEndpoints().each(function(endpoint) {
        for(i in endpoint.connections) {
            var source_filter = $(endpoint.connections[i].source).closest(".filter");
            var target_filter = $(endpoint.connections[i].target).closest(".filter");

            if(source_filter.closest(".filtergroup").find(".filtergroup-name").text() != target_filter.closest(".filtergroup").find(".filtergroup-name").text()) {
                var padding_count = source_filter.index() + 1 - target_filter.index();

                for(var j = 0; j < padding_count; j++) {
                    var new_filter = $(filter.toString());

                    target_filter.before(new_filter)
                    init_filter(new_filter, {"visible": false});
                }
            }
        }
    });
}

function pad_filter_groups(add_before) {
    var filter_groups = [];
    var filter_count = 0;
    var add_before = (add_before === undefined) ? false : add_before;

    $(".filtergroup-body").map(function (i) {
        filter_groups[i] = $(this).children().length;
    });

    filter_count = Math.max.apply(Math, filter_groups) - 2;

    $.each($(".filtergroup-body"), function(i) {
        var filters_to_add = filter_count - ($(this).children().length - 2);

        if(filters_to_add > 0) {
            for(var j = 0; j < filters_to_add; j++) {
                var new_filter = $(filter.toString());

                if(add_before) {
                    $(this).find(".add-filter-container.left").after(new_filter);
                } else {
                    $(this).find(".add-filter-container.right").before(new_filter);
                }

                init_filter(new_filter, {"visible": false});
            }
        }
    });
}

function init_filtergroup(filtergroup_selector, options) {
    var options = options || {};
    var name = options.name || "";

    $(filtergroup_selector).find(".filtergroup-name").text(name);
    $(filtergroup_selector).find(".add-filter-container.left").append($(filter_picker).clone(true));
    $(filtergroup_selector).find(".add-filter-container.right").append($(filter_picker).clone(true));
    $(filtergroup_selector).find(".dropdown-menu").last().addClass("pull-right");

    $(filtergroup_selector).find(".filtergroup-body").sortable({
        tolerance: "pointer",
        scroll: false,
        items: ".filter",
        cancel: ".sort-disabled",
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
            ui.item.closest(".filtergroup-body").children(".inactive").last().remove();
            pad_filter_groups();
        }
    });

    pad_filter_groups();
}

function init_filter(filter_selector, options) {
    var options = options || {};
    var visible = (options.visible === undefined) ? true : options.visible;
    var name = options.name || "";
    var type = options.type || "";
    var inputs = options.inputs || [];
    var outputs = options.outputs || [];

    update_sortable_filters();

    filter_selector.find(".filter-name").text(name);
    filter_selector.find(".filter-type").text(type);

    if(inputs.length > 0 && outputs.length > 0) {
        init_filter_inputs(filter_selector, inputs);
        init_filter_outputs(filter_selector, outputs);
    }

    filter_selector.click(function() {
        var filter_type = $(this).find(".filter-type").text();
        var filter_info = filter_metadata.filter(function (f) { return f.type == filter_type; })[0];

        $("#properties-container").children().show();

        if(selected_filter != null) {
            selected_filter.removeClass("selected-filter");
        }

        init_filter_properties(filter_info.type, filter_info.description, filter_info.parameters);

        $(this).addClass("selected-filter");
        selected_filter = $(this);
    });

    filter_selector.find(".delete-filter").click(function () {
        //TODO: Add modal confirmation
        remove_filter(filter_selector);
    });

    if(!visible) {
        filter_selector.addClass("inactive")
    }
}

function init_filter_inputs(filter_selector, inputs) {

    jsp.removeAllEndpoints(filter_selector.find(".filter-inputs"));

    for(i in inputs) {
        jsp.addEndpoint(filter_selector.find(".filter-inputs"),
            {
                uuid: "_" + filter_selector.closest(".filtergroup").find(".filtergroup-name").text() + "_" + filter_selector.find(".filter-name").text() + "_" + inputs[i].name,
                anchor:[0.5, (parseInt(i) + 1) * (1.0 / (inputs.length + 1)), -1, 0]
            },
            {
                endpoint: ["Dot", {radius: 5} ],
                cssClass: "input",
                hoverClass: "input-hover",
                paintStyle: { fillStyle: "#FF0000", opacity: 0.5 },
                isTarget: true,
                scope: inputs[i].type,
                overlays:[
                    [ "Label", { location: [0, -1.2], label: inputs[i].type, cssClass: "input-label" } ]
                ]
            }
        );
    }
}

function init_filter_outputs(filter_selector, outputs) {

    jsp.removeAllEndpoints(filter_selector.find(".filter-outputs"));

    for(i in outputs) {
        jsp.addEndpoint(filter_selector.find(".filter-outputs"),
            {
                uuid: "_" + filter_selector.closest(".filtergroup").find(".filtergroup-name").text() + "_" + filter_selector.find(".filter-name").text() + "_" + outputs[i].name,
                anchor:[0.5, (parseInt(i) + 1) * (1.0 / (outputs.length + 1)), 1, 0]
            },
            {
                endpoint: ["Dot", {radius: 5} ],
                cssClass: "output",
                hoverClass: "output-hover",
                paintStyle: { fillStyle: "#0000FF", opacity: 0.5 },
                isSource: true,
                scope: outputs[i].type,
                maxConnections: -1,
                connector: [ "Bezier" ],
                connectorStyle: {
                    gradient: { stops: [[0, "#00F"], [0.5, '#09098e'], [1, "#00F"]] },
                    strokeStyle: "#00F",
                    lineWidth: 5
                },
                overlays:[
                    [ "Label", { location: [0, -1.2], label: outputs[i].type, cssClass: "output-label" } ]
                ]
            }
        );
    }
}

function init_connections(inputs, filter_group_name, filter_name) {
    for(i in inputs) {
        var input_endpoint = jsp.getEndpoint(inputs[i].topic.replace(/\//g, '_')) || $("#" + inputs[i].topic.replace(/\//g, '_'));
        var output_endpoint = jsp.getEndpoint("_" + filter_group_name + "_" + filter_name + "_" + inputs[i].name);

        jsp.connect({ source: input_endpoint, target: output_endpoint});
    }
}

function remove_filter(filter_selector) {
    filter_selector.addClass("inactive");
    $(".filtergroup-body").children(".inactive:nth-child(" + (filter_selector.index() + 1) + ")").remove();
    pad_filter_groups();
    jsp.repaintEverything();
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

    $.each(topics, function(groupName, options) {
        var $optgroup = $("<optgroup>", {label: groupName});
        $optgroup.appendTo($("#" + topic_selector_id + "-topics"));

        $.each(options, function(index, option) {
            var $link = $("<a>", {text: option, class: "list-group-item"});
            var $option = $("<option>", {text: option, value: groupName});

            $option.appendTo($optgroup);
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
    var master = new WebSocket("ws://localhost:8888/master/");
    //var save = new WebSocket("ws://localhost:8888/save/");

    $("#properties-container").children().hide();

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

        $.each(filter_metadata, function(i, f) {
             $(filter_picker).append($('<li><a href="#">' + f.name + '</a></li>'));
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

    master.onmessage = function(evt) {
        load_filter_chain(JSON.parse(evt.data));
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
		Container:"gui-body"
	});

    jsp.doWhileSuspended(function() {
        jsp.bind("connection", function(info) {
            info.connection.getOverlay("label").setLabel(info.connection.id);
        });
    });
});