var jsp = null;
var selected_filter = null;
var filter = '<dd class="filter sort-disabled"><div class="filter-header"><select class="filter-picker"></select></div><div class="filter-body"><div class="filter-inputs"></div><div class="filter-outputs"></div></div></dd>';
var filtergroup = '<div class="filtergroup"><div class="filtergroup-header"></div><dl class="filtergroup-body"><dd class="filter-add left sort-disabled">◁ Add</dd>' + filter + '<dd class="filter-add right sort-disabled">Add ▷</dd></dl></div></div>';

function init_filtergroup(new_filtergroup) {
        $(new_filtergroup).find(".filtergroup-body").sortable({
        tolerance: "pointer",
        scroll: false,
        items: ".filter",
        cancel: ".sort-disabled,.ep",
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

    jsp.addEndpoint(new_filter.find(".filter-inputs"),
        { anchor:[0.5, 0.5, -1, 0] },
        {
            endpoint: ["Dot", {radius: 10} ],
            paintStyle: { fillStyle: "#FF0000", opacity: 0.5 },
            isTarget: true,
            scope: 'topic'
        }
    );

    jsp.addEndpoint(new_filter.find(".filter-outputs"),
        { anchor:[0.5, 0.5, 1, 0] },
        {
            endpoint: ["Dot", {radius: 10} ],
            paintStyle: { fillStyle: "#0000FF", opacity: 0.5 },
            isSource: true,
            scope: 'topic',
            connector: [ "StateMachine", {curviness: 100} ],
            connectorStyle: {
                strokeStyle: "#00FF00",
                lineWidth: 4
            }
        }
    );

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

function build_topic_list(topic_selector_id, data) {
    $("#" + topic_selector_id).empty();
    $("#" + topic_selector_id).append('<option value="None">None</option>');

    data = JSON.parse(data);
    topics = data.topics;
    selected = data.selected;

    $.each(topics, function(i, optgroups) {
        $.each(optgroups, function(groupName, options) {
            var $optgroup = $("<optgroup>", {label: groupName});
            $optgroup.appendTo($("#" + topic_selector_id));

            $.each(options, function(j, option) {
                var $option = $("<option>", {text: option, value: groupName});
                $option.appendTo($optgroup);
            });
        });
    });

    $("#" + topic_selector_id + " option").filter(function () { return $(this).html() == selected; }).val();
}

jsPlumb.ready(function() {
    var input1 = new WebSocket("ws://localhost:8888/input1/");
    var topics1 = new WebSocket("ws://localhost:8888/input1/topic/");
    var input2 = new WebSocket("ws://localhost:8888/input2/");
    var topics2 = new WebSocket("ws://localhost:8888/input1/topic/");

    draw_image("feed1", "None");
    draw_image("feed2", "None");

    input1.onmessage = function(evt) {
        draw_image("feed1", evt.data);
    };

    topics1.onmessage = function(evt) {
        build_topic_list("feed1-topics", evt.data);
    };

    input2.onmessage = function(evt) {
        draw_image("feed2", evt.data);
    };

    topics2.onmessage = function(evt) {
        build_topic_list("feed2-topics", evt.data);
    };

    $("#feed1-topics").change(function () {
        input1.send([$("option:selected", this).text(), $("option:selected", this).val()])
    });

    $("#feed2-topics").change(function () {
        input2.send([$("option:selected", this).text(), $("option:selected", this).val()])
    });

    $(window).resize(function() {
        jsp.repaintEverything();
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
        update_sortable_filters(new_filter);
        init_filter(new_filter);
    });

    $("body").on("click", ".filter-add.left", function() {
        var new_filter = $(filter.toString());

        $(this).after(new_filter);
        update_sortable_filters(new_filter);
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