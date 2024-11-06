---
layout: page
title: CHAT-429
permalink: /projects/chat429/
---

<style>
  .rounded-outline {
    color: #87d771;
    font-size: 40px;
    border: 8px solid #87d771; /* Border width and color */
    border-radius: 10px; /* Border radius for rounded corners */
    padding: 10px; /* Padding to create space between text and border */
    display: inline-block; /* Ensures that the border wraps around the text */
    margin: 0 auto; /* Center horizontally */
  }

  /* Center container for full-page centering */
  .center-container {
    text-align: center;
    height: 300px; /* Set the desired height */
    display: flex;
    align-items: center; /* Center vertically */
    justify-content: center; /* Center horizontally */
  }
</style>

<div class="center-container">
  <p class="rounded-outline">CHAT429</p>
</div>


## Overview
This project, developed in Go, uses TCP sockets to create a Discord-like program that allows users to connect to a server where they can join various text chats and talk to friends. it provides a framework similar to Discord in that there is an admin that can give roles to users and create and delete channels.