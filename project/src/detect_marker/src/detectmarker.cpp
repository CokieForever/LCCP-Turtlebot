<!DOCTYPE html>
<html lang='en'>
<head>
<meta charset='utf-8'>
<title>
group7/project/src/detect_marker/src/detectmarker.cpp at master - lcpp / group7 | 
GitLab
</title>
<link href="/assets/favicon-6e66d007b09473be55fec600ff017739.ico" rel="shortcut icon" type="image/vnd.microsoft.icon" />
<link href="/assets/application-1bb9d6f4be686c4a4b16debdf17fdf57.css" media="all" rel="stylesheet" />
<link href="/assets/print-8cff922ef0c4bb37621dbac136b43020.css" media="print" rel="stylesheet" />
<script src="/assets/application-f1f8edbae7e79cb56772eed9fa1d4a83.js"></script>
<meta content="authenticity_token" name="csrf-param" />
<meta content="25TGpZ34+FspAHZDovwIkTROtzmPWflJVVnmYwSTPsM=" name="csrf-token" />
<script type="text/javascript">
//<![CDATA[
window.gon={};gon.default_issues_tracker="gitlab";gon.api_version="v3";gon.api_token="2XZJKJbEyY2pQi4a5P8X";gon.gravatar_url="https://secure.gravatar.com/avatar/%{hash}?s=%{size}\u0026d=mm";gon.relative_url_root="";gon.gravatar_enabled=true;
//]]>
</script>
<meta name="viewport" content="width=device-width, initial-scale=1.0">




</head>

<body class='ui_mars project' data-page='projects:blob:show' data-project-id='103'>

<header class='navbar navbar-static-top navbar-gitlab'>
<div class='navbar-inner'>
<div class='container'>
<div class='app_logo'>
<span class='separator'></span>
<a class="home has_bottom_tooltip" href="/" title="Dashboard"><h1>GITLAB</h1>
</a><span class='separator'></span>
</div>
<h1 class='title'><span><a href="/groups/lcpp">lcpp</a> / group7</span></h1>
<button class='navbar-toggle' data-target='.navbar-collapse' data-toggle='collapse' type='button'>
<span class='sr-only'>Toggle navigation</span>
<i class='icon-reorder'></i>
</button>
<div class='navbar-collapse collapse'>
<ul class='nav navbar-nav'>
<li class='hidden-sm hidden-xs'>
<div class='search'>
<form accept-charset="UTF-8" action="/search" class="navbar-form pull-left" method="get"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /></div>
<input class="search-input" id="search" name="search" placeholder="Search in this project" type="text" />
<input id="group_id" name="group_id" type="hidden" />
<input id="project_id" name="project_id" type="hidden" value="103" />
<input id="search_code" name="search_code" type="hidden" value="true" />
<input id="repository_ref" name="repository_ref" type="hidden" value="master" />

<div class='search-autocomplete-opts hide' data-autocomplete-path='/search/autocomplete' data-autocomplete-project-id='103' data-autocomplete-project-ref='master'></div>
</form>

</div>

</li>
<li class='visible-sm visible-xs'>
<a class="has_bottom_tooltip" data-original-title="Search area" href="/search" title="Search"><i class='icon-search'></i>
</a></li>
<li>
<a class="has_bottom_tooltip" data-original-title="Help" href="/help" title="Help"><i class='icon-question-sign'></i>
</a></li>
<li>
<a class="has_bottom_tooltip" data-original-title="Public area" href="/public" title="Public area"><i class='icon-globe'></i>
</a></li>
<li>
<a class="has_bottom_tooltip" data-original-title="My snippets" href="/s/ga73kec" title="My snippets"><i class='icon-paste'></i>
</a></li>
<li>
<a class="has_bottom_tooltip" data-original-title="Profile settings&quot;" href="/profile" title="Profile settings"><i class='icon-user'></i>
</a></li>
<li>
<a class="has_bottom_tooltip" data-method="delete" data-original-title="Logout" href="/users/sign_out" rel="nofollow" title="Logout"><i class='icon-signout'></i>
</a></li>
<li class='hidden-xs'>
<a class="profile-pic" href="/u/ga73kec" id="profile-pic"><img alt="User activity" src="https://secure.gravatar.com/avatar/7b32f7266be3f088a89d8ec999ddfffa?s=26&amp;d=mm" />
</a></li>
</ul>
</div>
</div>
</div>
</header>

<script>
  GitLab.GfmAutoComplete.dataSource = "/lcpp/group7/autocomplete_sources?type=NilClass&type_id=master%2Fproject%2Fsrc%2Fdetect_marker%2Fsrc%2Fdetectmarker.cpp"
  GitLab.GfmAutoComplete.setup();
</script>

<div class='flash-container'>
</div>


<nav class='main-nav navbar-collapse collapse'>
<div class='container'><ul>
<li class="home"><a href="/lcpp/group7" title="Project"><i class='icon-home'></i>
</a></li><li class="active"><a href="/lcpp/group7/tree/master">Files</a>
</li><li class=""><a href="/lcpp/group7/commits/master">Commits</a>
</li><li class=""><a href="/lcpp/group7/network/master">Network</a>
</li><li class=""><a href="/lcpp/group7/graphs/master">Graphs</a>
</li><li class=""><a href="/lcpp/group7/issues">Issues
<span class='count issue_counter'>1</span>
</a></li><li class=""><a href="/lcpp/group7/merge_requests">Merge Requests
<span class='count merge_counter'>0</span>
</a></li><li class=""><a href="/lcpp/group7/wikis/home">Wiki</a>
</li></ul>
</div>
</nav>
<div class='container'>
<div class='content'><div class='tree-ref-holder'>
<form accept-charset="UTF-8" action="/lcpp/group7/refs/switch" class="project-refs-form" method="get"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /></div>
<select class="project-refs-select select2 select2-sm" id="ref" name="ref"><optgroup label="Branches"><option value="ga25rid">ga25rid</option>
<option value="ga35non">ga35non</option>
<option value="ga59muq">ga59muq</option>
<option value="ga73kec">ga73kec</option>
<option value="ga76jug">ga76jug</option>
<option selected="selected" value="master">master</option>
<option value="task1">task1</option></optgroup><optgroup label="Tags"></optgroup></select>
<input id="destination" name="destination" type="hidden" value="blob" />
<input id="path" name="path" type="hidden" value="project/src/detect_marker/src/detectmarker.cpp" />
</form>


</div>
<div class='tree-holder' id='tree-holder'>
<ul class='breadcrumb'>
<li>
<i class='icon-angle-right'></i>
<a href="/lcpp/group7/tree/master">group7
</a></li>
<li>
<a href="/lcpp/group7/tree/master/project">project</a>
</li>
<li>
<a href="/lcpp/group7/tree/master/project/src">src</a>
</li>
<li>
<a href="/lcpp/group7/tree/master/project/src/detect_marker">detect_marker</a>
</li>
<li>
<a href="/lcpp/group7/tree/master/project/src/detect_marker/src">src</a>
</li>
<li>
<a href="/lcpp/group7/blob/master/project/src/detect_marker/src/detectmarker.cpp"><span class='cblue'>
detectmarker.cpp
</span>
</a></li>
</ul>
<ul class='blob-commit-info bs-callout bs-callout-info hidden-xs'>
<li class='commit js-toggle-container'>
<div class='commit-row-title'>
<a class="commit_short_id" href="/lcpp/group7/commit/52ad4253ae53ed6bf9b3ab5459a9219b4f08ad49">52ad4253a</a>
&nbsp;
<span class='str-truncated'>
<a class="commit-row-message" href="/lcpp/group7/commit/52ad4253ae53ed6bf9b3ab5459a9219b4f08ad49">1st working version of task1</a>
<a class='text-expander js-toggle-button'>...</a>
</span>
<a class="pull-right" href="/lcpp/group7/tree/52ad4253ae53ed6bf9b3ab5459a9219b4f08ad49">Browse Code »</a>
<div class='notes_count'>
</div>
</div>
<div class='commit-row-description js-toggle-content'>
<p>
Tested in lab.
<br />Robots are moving and detect all markers.</p>
</div>
<div class='commit-row-info'>
<a class="commit-author-link has_tooltip" data-original-title="cokie.forever@gmail.com" href="mailto:cokie.forever@gmail.com"><img alt="" class="avatar s16" src="https://secure.gravatar.com/avatar/e67fc19526e042cb42e758968e4218dd?s=16&amp;d=mm" width="16" /> <span class="commit-author-name">CokieForever</span></a>
<div class='committed_ago'>
<time class='time_ago' data-placement='top' data-toggle='tooltip' datetime='2016-01-15T19:47:03Z' title='Jan 15, 2016 8:47pm'>2016-01-15 20:47:03 +0100</time>
<script>$('.time_ago').timeago().tooltip()</script>
 &nbsp;
</div>
</div>
</li>

</ul>
<div class='tree-content-holder' id='tree-content-holder'>
<div class='file-holder'>
<div class='file-title clearfix'>
<i class='icon-file'></i>
<span class='file_name'>
detectmarker.cpp
<small>3.92 KB</small>
</span>
<span class='options hidden-xs'><div class='btn-group tree-btn-group'>
<a class="btn btn-small" href="/lcpp/group7/edit/master/project/src/detect_marker/src/detectmarker.cpp">edit</a>
<a class="btn btn-small" href="/lcpp/group7/raw/master/project/src/detect_marker/src/detectmarker.cpp" target="_blank">raw</a>
<a class="btn btn-small" href="/lcpp/group7/blame/master/project/src/detect_marker/src/detectmarker.cpp">blame</a>
<a class="btn btn-small" href="/lcpp/group7/commits/master/project/src/detect_marker/src/detectmarker.cpp">history</a>
<a class="remove-blob btn btn-small btn-remove" data-toggle="modal" href="#modal-remove-blob">remove
</a></div>
</span>
</div>
<div class='file-content code'>
<div class='highlighted-data white'>
<div class='line-numbers'>
<a href="#L1" id="L1" rel="#L1"><i class='icon-link'></i>
1
</a><a href="#L2" id="L2" rel="#L2"><i class='icon-link'></i>
2
</a><a href="#L3" id="L3" rel="#L3"><i class='icon-link'></i>
3
</a><a href="#L4" id="L4" rel="#L4"><i class='icon-link'></i>
4
</a><a href="#L5" id="L5" rel="#L5"><i class='icon-link'></i>
5
</a><a href="#L6" id="L6" rel="#L6"><i class='icon-link'></i>
6
</a><a href="#L7" id="L7" rel="#L7"><i class='icon-link'></i>
7
</a><a href="#L8" id="L8" rel="#L8"><i class='icon-link'></i>
8
</a><a href="#L9" id="L9" rel="#L9"><i class='icon-link'></i>
9
</a><a href="#L10" id="L10" rel="#L10"><i class='icon-link'></i>
10
</a><a href="#L11" id="L11" rel="#L11"><i class='icon-link'></i>
11
</a><a href="#L12" id="L12" rel="#L12"><i class='icon-link'></i>
12
</a><a href="#L13" id="L13" rel="#L13"><i class='icon-link'></i>
13
</a><a href="#L14" id="L14" rel="#L14"><i class='icon-link'></i>
14
</a><a href="#L15" id="L15" rel="#L15"><i class='icon-link'></i>
15
</a><a href="#L16" id="L16" rel="#L16"><i class='icon-link'></i>
16
</a><a href="#L17" id="L17" rel="#L17"><i class='icon-link'></i>
17
</a><a href="#L18" id="L18" rel="#L18"><i class='icon-link'></i>
18
</a><a href="#L19" id="L19" rel="#L19"><i class='icon-link'></i>
19
</a><a href="#L20" id="L20" rel="#L20"><i class='icon-link'></i>
20
</a><a href="#L21" id="L21" rel="#L21"><i class='icon-link'></i>
21
</a><a href="#L22" id="L22" rel="#L22"><i class='icon-link'></i>
22
</a><a href="#L23" id="L23" rel="#L23"><i class='icon-link'></i>
23
</a><a href="#L24" id="L24" rel="#L24"><i class='icon-link'></i>
24
</a><a href="#L25" id="L25" rel="#L25"><i class='icon-link'></i>
25
</a><a href="#L26" id="L26" rel="#L26"><i class='icon-link'></i>
26
</a><a href="#L27" id="L27" rel="#L27"><i class='icon-link'></i>
27
</a><a href="#L28" id="L28" rel="#L28"><i class='icon-link'></i>
28
</a><a href="#L29" id="L29" rel="#L29"><i class='icon-link'></i>
29
</a><a href="#L30" id="L30" rel="#L30"><i class='icon-link'></i>
30
</a><a href="#L31" id="L31" rel="#L31"><i class='icon-link'></i>
31
</a><a href="#L32" id="L32" rel="#L32"><i class='icon-link'></i>
32
</a><a href="#L33" id="L33" rel="#L33"><i class='icon-link'></i>
33
</a><a href="#L34" id="L34" rel="#L34"><i class='icon-link'></i>
34
</a><a href="#L35" id="L35" rel="#L35"><i class='icon-link'></i>
35
</a><a href="#L36" id="L36" rel="#L36"><i class='icon-link'></i>
36
</a><a href="#L37" id="L37" rel="#L37"><i class='icon-link'></i>
37
</a><a href="#L38" id="L38" rel="#L38"><i class='icon-link'></i>
38
</a><a href="#L39" id="L39" rel="#L39"><i class='icon-link'></i>
39
</a><a href="#L40" id="L40" rel="#L40"><i class='icon-link'></i>
40
</a><a href="#L41" id="L41" rel="#L41"><i class='icon-link'></i>
41
</a><a href="#L42" id="L42" rel="#L42"><i class='icon-link'></i>
42
</a><a href="#L43" id="L43" rel="#L43"><i class='icon-link'></i>
43
</a><a href="#L44" id="L44" rel="#L44"><i class='icon-link'></i>
44
</a><a href="#L45" id="L45" rel="#L45"><i class='icon-link'></i>
45
</a><a href="#L46" id="L46" rel="#L46"><i class='icon-link'></i>
46
</a><a href="#L47" id="L47" rel="#L47"><i class='icon-link'></i>
47
</a><a href="#L48" id="L48" rel="#L48"><i class='icon-link'></i>
48
</a><a href="#L49" id="L49" rel="#L49"><i class='icon-link'></i>
49
</a><a href="#L50" id="L50" rel="#L50"><i class='icon-link'></i>
50
</a><a href="#L51" id="L51" rel="#L51"><i class='icon-link'></i>
51
</a><a href="#L52" id="L52" rel="#L52"><i class='icon-link'></i>
52
</a><a href="#L53" id="L53" rel="#L53"><i class='icon-link'></i>
53
</a><a href="#L54" id="L54" rel="#L54"><i class='icon-link'></i>
54
</a><a href="#L55" id="L55" rel="#L55"><i class='icon-link'></i>
55
</a><a href="#L56" id="L56" rel="#L56"><i class='icon-link'></i>
56
</a><a href="#L57" id="L57" rel="#L57"><i class='icon-link'></i>
57
</a><a href="#L58" id="L58" rel="#L58"><i class='icon-link'></i>
58
</a><a href="#L59" id="L59" rel="#L59"><i class='icon-link'></i>
59
</a><a href="#L60" id="L60" rel="#L60"><i class='icon-link'></i>
60
</a><a href="#L61" id="L61" rel="#L61"><i class='icon-link'></i>
61
</a><a href="#L62" id="L62" rel="#L62"><i class='icon-link'></i>
62
</a><a href="#L63" id="L63" rel="#L63"><i class='icon-link'></i>
63
</a><a href="#L64" id="L64" rel="#L64"><i class='icon-link'></i>
64
</a><a href="#L65" id="L65" rel="#L65"><i class='icon-link'></i>
65
</a><a href="#L66" id="L66" rel="#L66"><i class='icon-link'></i>
66
</a><a href="#L67" id="L67" rel="#L67"><i class='icon-link'></i>
67
</a><a href="#L68" id="L68" rel="#L68"><i class='icon-link'></i>
68
</a><a href="#L69" id="L69" rel="#L69"><i class='icon-link'></i>
69
</a><a href="#L70" id="L70" rel="#L70"><i class='icon-link'></i>
70
</a><a href="#L71" id="L71" rel="#L71"><i class='icon-link'></i>
71
</a><a href="#L72" id="L72" rel="#L72"><i class='icon-link'></i>
72
</a><a href="#L73" id="L73" rel="#L73"><i class='icon-link'></i>
73
</a><a href="#L74" id="L74" rel="#L74"><i class='icon-link'></i>
74
</a><a href="#L75" id="L75" rel="#L75"><i class='icon-link'></i>
75
</a><a href="#L76" id="L76" rel="#L76"><i class='icon-link'></i>
76
</a><a href="#L77" id="L77" rel="#L77"><i class='icon-link'></i>
77
</a><a href="#L78" id="L78" rel="#L78"><i class='icon-link'></i>
78
</a><a href="#L79" id="L79" rel="#L79"><i class='icon-link'></i>
79
</a><a href="#L80" id="L80" rel="#L80"><i class='icon-link'></i>
80
</a><a href="#L81" id="L81" rel="#L81"><i class='icon-link'></i>
81
</a><a href="#L82" id="L82" rel="#L82"><i class='icon-link'></i>
82
</a><a href="#L83" id="L83" rel="#L83"><i class='icon-link'></i>
83
</a><a href="#L84" id="L84" rel="#L84"><i class='icon-link'></i>
84
</a><a href="#L85" id="L85" rel="#L85"><i class='icon-link'></i>
85
</a><a href="#L86" id="L86" rel="#L86"><i class='icon-link'></i>
86
</a><a href="#L87" id="L87" rel="#L87"><i class='icon-link'></i>
87
</a><a href="#L88" id="L88" rel="#L88"><i class='icon-link'></i>
88
</a><a href="#L89" id="L89" rel="#L89"><i class='icon-link'></i>
89
</a><a href="#L90" id="L90" rel="#L90"><i class='icon-link'></i>
90
</a><a href="#L91" id="L91" rel="#L91"><i class='icon-link'></i>
91
</a><a href="#L92" id="L92" rel="#L92"><i class='icon-link'></i>
92
</a><a href="#L93" id="L93" rel="#L93"><i class='icon-link'></i>
93
</a><a href="#L94" id="L94" rel="#L94"><i class='icon-link'></i>
94
</a><a href="#L95" id="L95" rel="#L95"><i class='icon-link'></i>
95
</a><a href="#L96" id="L96" rel="#L96"><i class='icon-link'></i>
96
</a><a href="#L97" id="L97" rel="#L97"><i class='icon-link'></i>
97
</a><a href="#L98" id="L98" rel="#L98"><i class='icon-link'></i>
98
</a><a href="#L99" id="L99" rel="#L99"><i class='icon-link'></i>
99
</a><a href="#L100" id="L100" rel="#L100"><i class='icon-link'></i>
100
</a><a href="#L101" id="L101" rel="#L101"><i class='icon-link'></i>
101
</a><a href="#L102" id="L102" rel="#L102"><i class='icon-link'></i>
102
</a><a href="#L103" id="L103" rel="#L103"><i class='icon-link'></i>
103
</a><a href="#L104" id="L104" rel="#L104"><i class='icon-link'></i>
104
</a><a href="#L105" id="L105" rel="#L105"><i class='icon-link'></i>
105
</a><a href="#L106" id="L106" rel="#L106"><i class='icon-link'></i>
106
</a><a href="#L107" id="L107" rel="#L107"><i class='icon-link'></i>
107
</a><a href="#L108" id="L108" rel="#L108"><i class='icon-link'></i>
108
</a><a href="#L109" id="L109" rel="#L109"><i class='icon-link'></i>
109
</a><a href="#L110" id="L110" rel="#L110"><i class='icon-link'></i>
110
</a><a href="#L111" id="L111" rel="#L111"><i class='icon-link'></i>
111
</a><a href="#L112" id="L112" rel="#L112"><i class='icon-link'></i>
112
</a><a href="#L113" id="L113" rel="#L113"><i class='icon-link'></i>
113
</a><a href="#L114" id="L114" rel="#L114"><i class='icon-link'></i>
114
</a><a href="#L115" id="L115" rel="#L115"><i class='icon-link'></i>
115
</a><a href="#L116" id="L116" rel="#L116"><i class='icon-link'></i>
116
</a><a href="#L117" id="L117" rel="#L117"><i class='icon-link'></i>
117
</a><a href="#L118" id="L118" rel="#L118"><i class='icon-link'></i>
118
</a><a href="#L119" id="L119" rel="#L119"><i class='icon-link'></i>
119
</a><a href="#L120" id="L120" rel="#L120"><i class='icon-link'></i>
120
</a><a href="#L121" id="L121" rel="#L121"><i class='icon-link'></i>
121
</a><a href="#L122" id="L122" rel="#L122"><i class='icon-link'></i>
122
</a><a href="#L123" id="L123" rel="#L123"><i class='icon-link'></i>
123
</a><a href="#L124" id="L124" rel="#L124"><i class='icon-link'></i>
124
</a><a href="#L125" id="L125" rel="#L125"><i class='icon-link'></i>
125
</a><a href="#L126" id="L126" rel="#L126"><i class='icon-link'></i>
126
</a><a href="#L127" id="L127" rel="#L127"><i class='icon-link'></i>
127
</a><a href="#L128" id="L128" rel="#L128"><i class='icon-link'></i>
128
</a><a href="#L129" id="L129" rel="#L129"><i class='icon-link'></i>
129
</a><a href="#L130" id="L130" rel="#L130"><i class='icon-link'></i>
130
</a><a href="#L131" id="L131" rel="#L131"><i class='icon-link'></i>
131
</a><a href="#L132" id="L132" rel="#L132"><i class='icon-link'></i>
132
</a><a href="#L133" id="L133" rel="#L133"><i class='icon-link'></i>
133
</a></div>
<div class='highlight'>
<pre><code>#include &quot;aruco/aruco.h&quot;
#include &quot;cv_bridge/cv_bridge.h&quot;
#include &lt;sensor_msgs/image_encodings.h&gt;

#include &quot;detectmarker.h&quot;
#include &quot;detect_marker/MarkerInfo.h&quot;
#include &quot;detect_marker/MarkersInfos.h&quot;

DetectMarker::DetectMarker(ros::NodeHandle&amp; nodeHandle): m_nodeHandle(nodeHandle)
{
    ROS_INFO(&quot;Subscribing to camera image topic...&quot;);
    m_cameraSub = m_nodeHandle.subscribe(&quot;/camera/rgb/image_raw&quot;, 1, &amp;DetectMarker::cameraSubCallback, this);
    ros::Rate loopRate(10);
	while (ros::ok() &amp;&amp; m_cameraSub.getNumPublishers() &lt;= 0)
        loopRate.sleep();

    ROS_INFO(&quot;Creating markers topic...&quot;);
    m_markersPub = m_nodeHandle.advertise&lt;detect_marker::MarkersInfos&gt;(&quot;/markerinfo&quot;, 10);
    /*while (ros::ok() &amp;&amp; m_markersPub.getNumSubscribers() &lt;= 0)
        loopRate.sleep();*/
    
    ROS_INFO(&quot;Done, everything's ready.&quot;);
}

void DetectMarker::cameraSubCallback(const sensor_msgs::ImageConstPtr&amp; msg)
{
    ROS_INFO(&quot;Received image from camera.&quot;);

    cv::Mat img;
    try
    {
        cv_bridge::CvImageConstPtr imgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (imgPtr == NULL)
        {
            ROS_WARN(&quot;Received NULL image.&quot;);
            return;
        }
        img = imgPtr-&gt;image;
    }
    catch (cv_bridge::Exception&amp; ex)
    {
        ROS_ERROR(&quot;cv_bridge exception: %s&quot;, ex.what());
        exit(-1);
    }

    int width = img.cols;
    int height = img.rows;
    int channels = img.channels();
    ROS_INFO(&quot;Image size: %dx%dx%d&quot;, width, height, channels);
    if (img.cols &lt;= 0 || img.rows &lt;= 0)
    {
        ROS_WARN(&quot;Received emtpy / unconventional image.&quot;);
        return;
    }
    
    aruco::MarkerDetector detector;
    std::vector&lt;aruco::Marker&gt; markers;
    
    cv::Scalar colorScalar(255,155,0, 0);
    detector.detect(img, markers);

    int nbMarkers = markers.size();
    ROS_INFO(&quot;Amount of markers: %d&quot;, nbMarkers);

    detect_marker::MarkersInfos markersInfos;
    cv::Mat frame = img;
    for (int i=0 ; i &lt; nbMarkers ; i++)
    {
        aruco::Marker&amp; marker = markers[i];
        marker.draw(frame, colorScalar);

        Point center;
        Point corners[4];
        for (int j=0 ; j &lt; 4 ; j++)
        {
            corners[j].x = marker[j].x;
            corners[j].y = marker[j].y;
        }
        
        if (!ComputerQuadrilateralCenter(corners, &amp;center))
            ROS_WARN(&quot;Unable to compute center.&quot;);
        else
        {
            detect_marker::MarkerInfo markerInfo;
            markerInfo.x = 2*(center.x/(double)width)-1;
            markerInfo.y = 2*(center.y/(double)height)-1;
            markerInfo.id = marker.id;
            markersInfos.infos.push_back(markerInfo);

            ROS_INFO(&quot;Marker %d: (%.3f, %.3f)&quot;, markerInfo.id, markerInfo.x, markerInfo.y);
        }
    }
    
    ROS_INFO(&quot;Publishing %lu marker infos.&quot;, markersInfos.infos.size());
    m_markersPub.publish(markersInfos);

    cv::imshow(&quot;Marker Detection&quot;, frame);
    cv::waitKey(1);
}

bool DetectMarker::ComputeLinesIntersection(Point linePoints1[2], Point linePoints2[2], Point *isectPoint)
{
    double x1 = linePoints1[0].x;
    double x2 = linePoints1[1].x;
    double y1 = linePoints1[0].y;
    double y2 = linePoints1[1].y;    

    double x3 = linePoints2[0].x;
    double x4 = linePoints2[1].x;
    double y3 = linePoints2[0].y;
    double y4 = linePoints2[1].y;    

    double d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if (fabs(d) &lt; 1e-6)
        return false;

    isectPoint-&gt;x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
    isectPoint-&gt;y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
    return true;
}

bool DetectMarker::ComputerQuadrilateralCenter(Point points[4], Point *centerPoint)
{
    Point linePoints1[2] = {points[0], points[2]};
    Point linePoints2[2] = {points[1], points[3]};
    return ComputeLinesIntersection(linePoints1, linePoints2, centerPoint);
}

void DetectMarker::Detect()
{
    ROS_INFO(&quot;Starting detection.&quot;);
    ros::spin();
}</code></pre>
</div>
</div>

</div>

</div>
</div>

</div>
<div class='modal hide' id='modal-remove-blob'>
<div class='modal-dialog'>
<div class='modal-content'>
<div class='modal-header'>
<a class='close' data-dismiss='modal' href='#'>×</a>
<h3 class='page-title'>Remove detectmarker.cpp</h3>
<p class='light'>
From branch
<strong>master</strong>
</p>
</div>
<div class='modal-body'>
<form accept-charset="UTF-8" action="/lcpp/group7/blob/master/project/src/detect_marker/src/detectmarker.cpp" class="form-horizontal" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="_method" type="hidden" value="delete" /><input name="authenticity_token" type="hidden" value="25TGpZ34+FspAHZDovwIkTROtzmPWflJVVnmYwSTPsM=" /></div>
<div class='form-group commit_message-group'>
<label class="control-label" for="commit_message">Commit message
</label><div class='col-sm-10'>
<div class='commit-message-container'>
<div class='max-width-marker'>
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
</div>
<textarea class="form-control" id="commit_message" name="commit_message" placeholder="Removed this file because..." required="required" rows="3">
</textarea>
</div>

</div>
</div>
<div class='form-group'>
<div class='col-sm-2'></div>
<div class='col-sm-10'>
<input class="btn btn-remove" name="commit" type="submit" value="Remove file" />
<a class="btn btn-cancel" data-dismiss="modal" href="#">Cancel</a>
</div>
</div>
</form>

</div>
</div>
</div>
</div>

</div>
</div>
</body>
</html>
