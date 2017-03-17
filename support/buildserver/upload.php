<?php
	// see docs/Travis.md, .travis.sh

	$baseDir = "/var/www/builds/";

	$firmwareFile = $_FILES["file"];
	$manualFile = $_FILES["manual"];

	$recentCommits = $_POST["recent_commits"];
	$travisJobId = sanitize($_POST["travis_build_number"]);
	$lastCommitDate = sanitize($_POST["last_commit_date"]);
	$revision = sanitize($_POST["revision"]);
	$branch = sanitize($_POST["branch"]);

	$github_repo = sanitize($_POST["github_repo"]);
	$build_name = sanitize($_POST["build_name"]);

	$uploadDir = $baseDir . "/" . $github_repo . "/" . $lastCommitDate . "/";
	$prefix = $uploadDir . $travisJobId . "_" . $revision . "_" . $build_name;

	if(!file_exists($uploadDir)) mkdir($uploadDir, 0770, true);

	if($firmwareFile) {
		$uploadfile = $prefix . "_" . (basename($firmwareFile['name']));
		if(move_uploaded_file($firmwareFile['tmp_name'], $uploadfile)) {
			echo "upload succeeded.\n";
		}
		else {
			echo "upload failed $uploadfile\n";
		}
	}

	if($manualFile) {
		$uploadfile = $prefix . "_" . (basename($manualFile['name']));
		if(move_uploaded_file($manualFile['tmp_name'], $uploadfile)) {
			echo "upload succeeded.\n";
		}
		else {
			echo "upload failed $uploadfile\n";
		}
	}

	if($revision && $lastCommitDate && $recentCommits) {
		$changelog = fopen($prefix . "_changes.txt", "w") or die ("unable to open changelog file for writing");
		fwrite($changelog, $recentCommits);
		fclose($changelog);
	}

	print_r($_FILES);
	print_r($_POST);
	print_r($_GET);

	function sanitize($str) {
		return (preg_replace('/[^A-Za-z0-9_\-]/', '_', ($str)));
	}

?>
